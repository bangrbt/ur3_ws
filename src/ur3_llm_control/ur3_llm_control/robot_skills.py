#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Skills Library:
Tap hop cac ky nang nguyen thuy va nang cao (Primitive & Advanced Skills)
dieu khien robot UR3/UR3e thong qua MoveIt 2, ket hop dong bo vi tri vat the
vat ly tren Gazebo va RViz.

Cac skill co ban:
- home()
- pick(object)
- place(object, zone)
- move_above(target)
- move_to_zone(zone)
- open_gripper()
- close_gripper(object)

Cac skill nang cao:
- swap(object_a, object_b): Hoan doi 2 vat the su dung zone_temp
- stack(object_top, object_bottom): Xep chong vat the
- reset_scene(): Dua tat ca cac khoi hop ve lai 5 khay phoi ban dau
- inspect_scene(): Tra cuu trang thai toan bo khong gian lam viec

Moi skill tra ve trang thai thuc thi: SUCCESS, FAILED, INVALID_OBJECT, PLANNING_FAILED
"""

import os
import time
import math
import json
import subprocess
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import (
    PlanningScene,
    CollisionObject,
    AttachedCollisionObject,
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath, ApplyPlanningScene
from tf2_ros import Buffer, TransformListener


class RobotSkills:
    """Thu vien ky nang MoveIt 2 cho canh tay UR3 voi toc do nhanh, quy dao toi uu va chong chong de vat the."""

    def __init__(self, node: Node, scene_config: dict):
        self.node = node
        self.scene_config = scene_config

        self.table_config = scene_config.get("table", {})
        self.objects_config = scene_config.get("objects", {})
        self.zones_config = scene_config.get("zones", {})
        self.motion_params = scene_config.get("motion_params", {})

        # Toa do vi tri cac vat the hien tai (duoc cap nhat dong khi pick/place)
        self.object_positions = {}
        for obj_name, obj_data in self.objects_config.items():
            self.object_positions[obj_name] = list(obj_data.get("initial_position", [0.24, 0.0, 0.02]))

        # Theo doi vat the hien dang chiem giu tai tung Zone (de tranh chong de)
        self.zone_occupants = {z_name: None for z_name in self.zones_config}

        self.holding_object = None  # Vat the robot dang kep
        self._gazebo_joint_plugins = set()
        self._gazebo_robot_id = None
        self.grasp_offsets = {}

        # Action Clients MoveIt 2
        self._move_group_client = ActionClient(self.node, MoveGroup, "/move_action")
        self._execute_traj_client = ActionClient(self.node, ExecuteTrajectory, "/execute_trajectory")
        self._cartesian_path_client = self.node.create_client(GetCartesianPath, "/compute_cartesian_path")
        self._planning_scene_client = self.node.create_client(ApplyPlanningScene, "/apply_planning_scene")

        # Dynamic Cube State Publisher
        self.cube_state_pub = self.node.create_publisher(String, "/scene/cube_states", 10)
        # Gripper Command Publishers (ket noi truc tiep toi ros_gz_bridge -> Gazebo)
        self.gripper_left_pub = self.node.create_publisher(Float64, "/gripper/left_cmd", 10)
        self.gripper_right_pub = self.node.create_publisher(Float64, "/gripper/right_cmd", 10)

        # Gripper Joint State Publisher (phuc vu hien thi co bop muot ma lien tuc tren RViz)
        self.gripper_joint_pub = self.node.create_publisher(JointState, "/joint_states", 10)
        self.current_gripper_pos = 0.016
        self.target_gripper_pos = 0.016
        # Timer duy tri va noi suy chuyen dong co bop 20Hz deu dan
        self.gripper_state_timer = self.node.create_timer(0.05, self._publish_gripper_state)

        # Joint States Subscription & Tracking
        self.current_joint_positions = {}
        self.joint_sub = self.node.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 10
        )

        # Camera Perception Subscription
        self.camera_perception_state = {}
        self.last_camera_state_at = None
        self.camera_sub = self.node.create_subscription(
            String, "/scene/camera_state", self._camera_state_cb, 10
        )

        # TF Buffer & Listener for gripper tracking
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        # Timer dinh ky theo doi tool0 khi dang gap vat de cap nhat vi tri vat the
        self.tracking_timer = self.node.create_timer(0.1, self._tracking_callback)

        # Khoi tao gripper; Gazebo joint duoc tao chi tai luc gap cube
        self._init_gripper_hardware()

        # Orientation chu vi thang dung vuong goc mat ban (Top-down grasp)
        # Khau tac dong cuoi quay huong xuong ban: Pitch = 180 do
        self.top_down_quaternion = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)

        # Tham so chieu cao
        self.approach_height = float(self.motion_params.get("approach_height", 0.14))
        self.travel_height = float(self.motion_params.get("travel_height", 0.20))
        self.grasp_height = float(self.motion_params.get("grasp_height", 0.04))
        self.place_height = float(self.motion_params.get("place_height", 0.04))
        self.home_joints = list(self.motion_params.get("home_joints", [0.0, -1.3, 1.5, -1.7, -1.57, 0.0]))

        # Van toc toi da khi di chuyen duong thang Cartesian (tang toc do muot ma)
        self.cartesian_max_vel = float(self.motion_params.get("cartesian_speed", 1.2))

        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

    # =========================================================================
    # --- HELPER DIEU KHIEN GRIPPER VA DONG BO CAM BIEN ---
    # =========================================================================

    def _send_ignition_topic(self, topic: str, msg_type: str = "ignition.msgs.Empty", payload: str = "unused: true"):
        """Gui command qua Ignition Gazebo topic trong luong nen."""
        def _call():
            cmd = ["ign", "topic", "-t", topic, "-m", msg_type, "-p", payload]
            try:
                subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=0.8)
            except Exception:
                pass
        threading.Thread(target=_call, daemon=True).start()

    def _gazebo_model_id(self):
        if self._gazebo_robot_id is not None:
            return self._gazebo_robot_id
        import re
        try:
            result = subprocess.run(
                ["ign", "topic", "-e", "-t", "/world/default/pose/info", "-n", "1"],
                capture_output=True, text=True, timeout=4.0)
            match = re.search(r'name: "ur"\s+id: (\d+)', result.stdout)
            if match:
                self._gazebo_robot_id = int(match.group(1))
        except (OSError, subprocess.TimeoutExpired):
            pass
        return self._gazebo_robot_id

    def _gazebo_joint_topic(self, object_name, action):
        return f"/gripper/{object_name}/{action}"

    def _command_gazebo_joint(self, object_name, action):
        state_topic = self._gazebo_joint_topic(object_name, "state")
        watcher = None
        try:
            watcher = subprocess.Popen(
                ["ign", "topic", "-e", "-t", state_topic, "-n", "1"],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            time.sleep(0.8)
            sent = subprocess.run(
                ["ign", "topic", "-t", self._gazebo_joint_topic(object_name, action),
                 "-m", "ignition.msgs.Empty", "-p", "unused: true"],
                capture_output=True, text=True, timeout=5.0)
            if sent.returncode != 0:
                return False
            output, _ = watcher.communicate(timeout=6.0)
            return f'data: "{("attached" if action == "attach" else "detached")}"' in output
        except (OSError, subprocess.TimeoutExpired):
            return False
        finally:
            if watcher is not None and watcher.poll() is None:
                watcher.terminate()
                try:
                    watcher.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    watcher.kill()

    def _gazebo_attach(self, object_name):
        """Create a Gazebo fixed joint only while fingers surround the cube."""
        if object_name in self._gazebo_joint_plugins:
            for attempt in range(3):
                if self._command_gazebo_joint(object_name, "attach"):
                    return True
                time.sleep(0.3)
            self.node.get_logger().error(f"Gazebo khong xac nhan reattach {object_name}")
            return False
        robot_id = self._gazebo_model_id()
        if robot_id is None:
            return False
        xml = (f"<parent_link>wrist_3_link</parent_link>"
               f"<child_model>{object_name}</child_model><child_link>link</child_link>"
               f"<detach_topic>{self._gazebo_joint_topic(object_name, 'detach')}</detach_topic>"
               f"<attach_topic>{self._gazebo_joint_topic(object_name, 'attach')}</attach_topic>"
               f"<output_topic>{self._gazebo_joint_topic(object_name, 'state')}</output_topic>")
        request = (f'entity {{ id: {robot_id} type: MODEL }} plugins {{ '
                   'name: "ignition::gazebo::systems::DetachableJoint" '
                   'filename: "ignition-gazebo-detachable-joint-system" '
                   f'innerxml: {json.dumps(xml)} }}')
        try:
            result = subprocess.run(
                ["ign", "service", "-s", "/world/default/entity/system/add",
                 "--reqtype", "ignition.msgs.EntityPlugin_V",
                 "--reptype", "ignition.msgs.Boolean", "--timeout", "3000",
                 "--req", request], capture_output=True, text=True, timeout=5.0)
            if result.returncode != 0 or "data: true" not in result.stdout:
                return False
            for _ in range(15):
                info = subprocess.run(
                    ["ign", "topic", "-i", "-t", self._gazebo_joint_topic(object_name, "state")],
                    capture_output=True, text=True, timeout=3.0)
                if "ignition.msgs.StringMsg" in info.stdout:
                    self._gazebo_joint_plugins.add(object_name)
                    return True
                time.sleep(0.1)
        except (OSError, subprocess.TimeoutExpired):
            return False
        return False

    def _gazebo_detach(self, object_name):
        if object_name not in self._gazebo_joint_plugins:
            return False
        for attempt in range(3):
            if self._command_gazebo_joint(object_name, "detach"):
                return True
            time.sleep(0.3)
        self.node.get_logger().error(f"Gazebo khong xac nhan detach {object_name}")
        return False

    def _send_gripper_joint_cmd(self, position_left: float, position_right: float):
        """Publish joint position command de dieu khien ngon tay gripper vat ly qua ros_gz_bridge."""
        try:
            msg_l = Float64(data=float(position_left))
            msg_r = Float64(data=float(position_right))
            self.gripper_left_pub.publish(msg_l)
            self.gripper_right_pub.publish(msg_r)
        except Exception:
            pass

    def _init_gripper_hardware(self):
        """Mo rong ngon tay kep Industrial Gripper san sang lam viec."""
        def _async_init():
            time.sleep(1.0)
            self.target_gripper_pos = 0.016
            self.current_gripper_pos = 0.016
            self._send_gripper_joint_cmd(0.016, -0.016)
        threading.Thread(target=_async_init, daemon=True).start()

    def _camera_state_cb(self, msg: String):
        """Use only fresh, complete observations as planning state."""
        try:
            data = json.loads(msg.data)
            self.camera_perception_state = data
            self.last_camera_state_at = time.monotonic()
            detected = data.get("detected_objects", {})
            expected = len(self.objects_config) - int(self.holding_object is not None)
            if not data.get("camera_active") or len(detected) < expected:
                return
            for obj_name, info in detected.items():
                if self.holding_object != obj_name and "x" in info and "y" in info:
                    self.object_positions[obj_name] = [info["x"], info["y"], 0.02]
            zones = data.get("zone_occupants", {})
            for zone, occupant in zones.items():
                if zone in self.zone_occupants and occupant != self.holding_object:
                    self.zone_occupants[zone] = occupant
            self._publish_dynamic_cube_state()
        except (ValueError, TypeError, KeyError) as exc:
            self.node.get_logger().warn(f"Camera state khong hop le: {exc}")

    def _joint_state_cb(self, msg: JointState):
        """Cap nhat gia tri khop hien tai."""
        for name, pos in zip(msg.name, msg.position):
            self.current_joint_positions[name] = pos

    def _tracking_callback(self):
        """Khi dang gap vat, cap nhat toa do vat the theo dau tay kep tool0."""
        if self.holding_object:
            try:
                t = self.tf_buffer.lookup_transform("base_link", "tool0", rclpy.time.Time())
                tx = t.transform.translation.x
                ty = t.transform.translation.y
                tz = max(0.02, t.transform.translation.z - 0.08)
                self.object_positions[self.holding_object] = [tx, ty, tz]
                self._publish_dynamic_cube_state()
            except Exception:
                pass

    def _publish_dynamic_cube_state(self):
        """Phat trang thai vi tri cac hop len topic de SceneSpawner cap nhat RViz."""
        try:
            msg = String()
            msg.data = json.dumps(self.object_positions)
            self.cube_state_pub.publish(msg)
        except Exception:
            pass

    def _unwrap_joint_target(self, goal_joints: list) -> list:
        """Keep configured joint targets inside URDF limits.

        The UR3 joints are bounded, so a modulo-2π shortcut can turn a valid
        home pose into an unreachable or self-colliding one. MoveIt plans the
        shortest collision-free route to these exact, configured values.
        """
        return [float(value) for value in goal_joints]

    # =========================================================================
    # --- CAC SKILL NGUYEN THUY ---
    # =========================================================================

    def home(self) -> str:
        """Dua tay may ve tu the home hop le; MoveIt chon duong di tranh va cham."""
        self.node.get_logger().info("Thuc thi Skill: home()")
        for attempt in range(3):
            if self._move_to_joint_target(self.home_joints):
                return "SUCCESS"
            self.node.get_logger().warn(f"Home planning attempt {attempt + 1}/3 failed; retrying")
            time.sleep(0.25)
        return "PLANNING_FAILED"

    def _publish_gripper_state(self):
        """Noi suy vi tri ngon tay va publish lien tuc tren RViz (/joint_states) va Gazebo."""
        step = 0.0025
        if abs(self.current_gripper_pos - self.target_gripper_pos) > 1e-4:
            if self.current_gripper_pos < self.target_gripper_pos:
                self.current_gripper_pos = min(self.target_gripper_pos, self.current_gripper_pos + step)
            else:
                self.current_gripper_pos = max(self.target_gripper_pos, self.current_gripper_pos - step)
            self._send_gripper_joint_cmd(self.current_gripper_pos, -self.current_gripper_pos)

        try:
            js = JointState()
            js.header.stamp = self.node.get_clock().now().to_msg()
            js.name = ["gripper_left_joint", "gripper_right_joint"]
            js.position = [float(self.current_gripper_pos), float(-self.current_gripper_pos)]
            self.gripper_joint_pub.publish(js)
        except Exception:
            pass

    def _animate_gripper(self, target_pos: float, steps: int = 12, duration: float = 0.4):
        """Send gradual position commands so Gazebo fingers contact the cube gently."""
        start = self.current_gripper_pos
        for index in range(1, steps + 1):
            position = start + (target_pos - start) * index / steps
            self.current_gripper_pos = position
            self._send_gripper_joint_cmd(position, -position)
            time.sleep(duration / steps)
        self.target_gripper_pos = target_pos

    def open_gripper(self) -> str:
        """Mo rong ngon tay kep Industrial Gripper de san sang gap vat hoac nha vat tren khong."""
        self.node.get_logger().info("Thuc thi Skill: open_gripper() - Mo rong ngon tay kep")
        held = self.holding_object
        if held and not self._gazebo_detach(held):
            return "GRIPPER_FAILED"
        if held:
            time.sleep(0.20)
        self._animate_gripper(0.016, steps=10, duration=0.30)
        if held:
            if not self._detach_object_from_robot(held):
                return "SCENE_FAILED"
            if not self.sync_collision_scene(exclude=held):
                return "SCENE_FAILED"
            self.holding_object = None
        time.sleep(0.15)
        return "SUCCESS"

    def release_object_in_tray(self) -> str:
        """Mo nhe ngon tay kep du de tha khoi hop ma hoan toan khong cham vao thanh khay."""
        self.node.get_logger().info("Thuc thi: release_object_in_tray() - Nha vat nhe nhang trong khay")
        # Thao joint khi cube da tua tren khay, roi mo ngon tu tu de tranh bat vat.
        held = self.holding_object
        if held and not self._gazebo_detach(held):
            return "GRIPPER_FAILED"
        if held:
            time.sleep(0.20)
        self._animate_gripper(0.016, steps=12, duration=0.45)
        if held:
            if not self._detach_object_from_robot(held):
                return "SCENE_FAILED"
            if not self.sync_collision_scene(exclude=held):
                return "SCENE_FAILED"
            self.holding_object = None
        time.sleep(0.40)
        return "SUCCESS"

    def close_gripper(self, object_name: str = None) -> str:
        """Co bop ngon tay kep Industrial Gripper ep chat vat the bang luc ma sat."""
        self.node.get_logger().info(f"Thuc thi Skill: close_gripper(object={object_name}) - Co bop ngon kep")
        # 0.008m de hai mat silicone vua cham cube 4cm, khong nen vat.
        target = 0.008 if object_name else 0.000
        self._animate_gripper(target, steps=12, duration=0.40)
        if object_name:
            tool_xy = None
            for _ in range(5):
                try:
                    transform = self.tf_buffer.lookup_transform(
                        "base_link", "tool0", rclpy.time.Time())
                    tool_xy = (transform.transform.translation.x,
                               transform.transform.translation.y)
                    break
                except Exception:
                    time.sleep(0.1)
            observed = self.object_positions.get(object_name)
            if tool_xy is None or observed is None:
                return "GRASP_POSE_UNKNOWN"
            offset = (observed[0] - tool_xy[0], observed[1] - tool_xy[1])
            if math.hypot(*offset) > 0.015:
                self.node.get_logger().error(f"Gripper lech {math.hypot(*offset):.3f}m so voi {object_name}")
                return "GRASP_MISALIGNED"
            self.grasp_offsets[object_name] = offset
            if not self._gazebo_attach(object_name):
                self.node.get_logger().error(f"Gazebo attach that bai voi {object_name}")
                return "GRIPPER_FAILED"
            if not self._attach_object_to_robot(object_name):
                self._gazebo_detach(object_name)
                return "SCENE_FAILED"
            self.holding_object = object_name
            for zone, occupant in self.zone_occupants.items():
                if occupant == object_name:
                    self.zone_occupants[zone] = None
        time.sleep(0.20)
        return "SUCCESS"

    def _tool_reached(self, target_pose: Pose, timeout_sec: float = 8.0) -> bool:
        """Confirm the measured TCP reached a grasp/place goal before touching a cube."""
        deadline = time.monotonic() + timeout_sec
        last_error = None
        while time.monotonic() < deadline:
            try:
                transform = self.tf_buffer.lookup_transform(
                    "base_link", "tool0", rclpy.time.Time())
                actual = transform.transform.translation
                xy_error = math.hypot(actual.x - target_pose.position.x,
                                      actual.y - target_pose.position.y)
                z_error = abs(actual.z - target_pose.position.z - 0.082)
                last_error = (xy_error, z_error)
                if xy_error <= 0.012 and z_error <= 0.015:
                    return True
            except Exception:
                pass
            time.sleep(0.15)
        self.node.get_logger().warn(f"TCP chua toi dich: sai so XY/Z = {last_error}")
        return False

    def _travel_to(self, x: float, y: float) -> bool:
        """Move directly at transfer height; use sampling planning only as fallback."""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = self.travel_height
        pose.orientation = self.top_down_quaternion
        # The workspace is compact and all transfers happen above the cubes.
        # A full collision-checked Cartesian segment is shorter and preserves
        # the current IK branch, preventing unnecessary wrist/shoulder loops.
        if self._move_cartesian([pose]):
            return True
        self.node.get_logger().warn(
            "Cartesian transfer khong hop le; chuyen sang MoveGroup tranh va cham"
        )
        if self._move_to_pose_target(pose):
            return True
        if self.holding_object is None and self.home() == "SUCCESS":
            return self._move_cartesian([pose]) or self._move_to_pose_target(pose)
        return False

    def move_above(self, target_name: str) -> str:
        pos = self._get_target_xy(target_name)
        if pos is None:
            return "INVALID_OBJECT"
        return "SUCCESS" if self._travel_to(pos[0], pos[1]) else "PLANNING_FAILED"

    def move_to_zone(self, zone_name: str) -> str:
        """Di chuyen tay kep den phia tren vung Zone."""
        self.node.get_logger().info(f"Thuc thi Skill: move_to_zone({zone_name})")
        return self.move_above(zone_name)

    def pick(self, object_name: str) -> str:
        """
        Chu trinh gap vat hoan chinh:
        1. Kiem tra tinh hop le
        2. Giai phong zone dang giu vat (neu co)
        3. Mo kep
        4. Tiep can tren khong (Cartesian nhanh)
        5. Ha kep Cartesian xuong vat (thang dung tuyet doi)
        6. Dong kep (Dinh kem vat the & dong bo vi tri Gazebo)
        7. Nhac vat thang dung len do cao chuyen ngang
        """
        self.node.get_logger().info(f"Thuc thi Skill: pick({object_name})")
        if object_name not in self.objects_config:
            self.node.get_logger().error(f"pick: Vat the '{object_name}' khong ton tai!")
            return "INVALID_OBJECT"

        if self.holding_object is not None:
            self.node.get_logger().error(f"pick: Robot dang giu '{self.holding_object}', khong the gap them!")
            return "FAILED"

        obj_xy = self.object_positions.get(object_name)
        if obj_xy is None:
            return "INVALID_OBJECT"

        # Giai phong khoi zone cu neu vat tung duoc dat o zone do
        for z, occupant in list(self.zone_occupants.items()):
            if occupant == object_name:
                self.zone_occupants[z] = None

        # 1. Mo kep va loai target khoi collision scene de co the cham vao no.
        self.open_gripper()
        if not self.sync_collision_scene(exclude=object_name):
            return "SCENE_FAILED"

        # 2. Tiep can tren khong cua vat
        approach_pose = Pose()
        approach_pose.position.x = obj_xy[0]
        approach_pose.position.y = obj_xy[1]
        approach_pose.position.z = self.approach_height
        approach_pose.orientation = self.top_down_quaternion

        if not self._travel_to(obj_xy[0], obj_xy[1]):
            return "PLANNING_FAILED"
        if not self._move_cartesian([approach_pose]):
            if not self._move_to_pose_target(approach_pose):
                return "PLANNING_FAILED"

        # 3. Ha kep Cartesian xuong vat
        grasp_pose = Pose()
        grasp_pose.position.x = obj_xy[0]
        grasp_pose.position.y = obj_xy[1]
        grasp_pose.position.z = self.grasp_height
        grasp_pose.orientation = self.top_down_quaternion

        if not self._move_cartesian([grasp_pose]):
            if not self._move_to_pose_target(grasp_pose):
                return "PLANNING_FAILED"
        if not self._tool_reached(grasp_pose):
            if not self._move_to_pose_target(grasp_pose) or not self._tool_reached(grasp_pose):
                return "GRASP_MISALIGNED"

        # Tam dung on dinh de tranh va cham nay vat
        time.sleep(0.25)

        # 4. Dong kep co bop kẹp chặt vật thể bằng lực ma sát
        close_status = self.close_gripper(object_name)
        if close_status != "SUCCESS":
            return close_status
        time.sleep(0.15)
        self._publish_dynamic_cube_state()

        # 5. Nhac vat thang dung toi do cao chuyen ngang truoc khi doi vi tri.
        # Tu the thap co the lam cube dang cam cham forearm khi MoveGroup gap tay.
        lift_pose = Pose()
        lift_pose.position.x = obj_xy[0]
        lift_pose.position.y = obj_xy[1]
        lift_pose.position.z = self.travel_height
        lift_pose.orientation = self.top_down_quaternion
        if not self._move_cartesian([lift_pose]):
            if not self._move_to_pose_target(lift_pose):
                return "RETRACT_FAILED"

        self.object_positions[object_name] = [obj_xy[0], obj_xy[1], self.travel_height]
        self._publish_dynamic_cube_state()

        return "SUCCESS"

    def place(self, object_name: str, zone_name: str) -> str:
        """
        Chu trinh dat vat hoan chinh bang gripper vat ly:
        1. Kiem tra tinh hop le
        2. Tinh toan toa do dat (tu dong tranh chong de neu o da co vat khac)
        3. Di chuyen tren khong toi Zone (Cartesian Transfer ngang)
        4. Ha vat xuong mat ban (Cartesian Descend)
        5. Mo kep (Nha DetachableJoint & tha vat that trong Gazebo)
        6. Nhac kep len cao (Cartesian Retract)
        7. Cap nhat toa do vat the va trang thai zone
        """
        self.node.get_logger().info(f"Thuc thi Skill: place({object_name}, {zone_name})")
        if object_name not in self.objects_config:
            return "INVALID_OBJECT"

        if zone_name not in self.zones_config:
            return "INVALID_OBJECT"

        if self.holding_object != object_name:
            self.node.get_logger().error(f"place: Robot khong giu vat '{object_name}' (hien tai: '{self.holding_object}')!")
            return "FAILED"

        zone_data = self.zones_config[zone_name]
        zone_pos = zone_data.get("position", [0.35, 0.0, 0.001])

        occupied_by = self.zone_occupants.get(zone_name)
        if occupied_by not in (None, object_name):
            return "ZONE_OCCUPIED"
        offset_x, offset_y = self.grasp_offsets.get(object_name, (0.0, 0.0))
        target_x = zone_pos[0] - offset_x
        target_y = zone_pos[1] - offset_y

        # 1. Tiep can tren khong cua Zone (Di chuyen duong thang Cartesian ngang)
        zone_approach_pose = Pose()
        zone_approach_pose.position.x = target_x
        zone_approach_pose.position.y = target_y
        zone_approach_pose.position.z = self.approach_height
        zone_approach_pose.orientation = self.top_down_quaternion

        if not self._travel_to(target_x, target_y):
            return "PLANNING_FAILED"
        if not self._move_cartesian([zone_approach_pose]):
            if not self._move_to_pose_target(zone_approach_pose):
                return "PLANNING_FAILED"

        self.object_positions[object_name] = [target_x, target_y, self.approach_height]
        self._publish_dynamic_cube_state()

        # 2. Ha dat vat xuong mat ban (Cartesian thang dung)
        place_pose = Pose()
        place_pose.position.x = target_x
        place_pose.position.y = target_y
        place_pose.position.z = self.place_height
        place_pose.orientation = self.top_down_quaternion

        if not self._move_cartesian([place_pose]):
            if not self._move_to_pose_target(place_pose):
                return "PLANNING_FAILED"
        if not self._tool_reached(place_pose):
            if not self._move_to_pose_target(place_pose) or not self._tool_reached(place_pose):
                return "PLANNING_FAILED"

        # Cho tay va cube het dao dong truoc khi tach joint.
        time.sleep(0.80)

        # 3. Nha vat nhe nhang trong khay ma khong quet vao thanh khay
        release_status = self.release_object_in_tray()
        if release_status != "SUCCESS":
            return release_status

        final_pos = [target_x, target_y, 0.02]
        self.object_positions[object_name] = final_pos
        self.zone_occupants[zone_name] = object_name
        self._publish_dynamic_cube_state()
        # 4. Nhac kep thang dung len tren khong (Cartesian Retract)
        if not self._move_cartesian([zone_approach_pose]):
            if not self._move_to_pose_target(zone_approach_pose):
                return "RETRACT_FAILED"

        # Raise to transfer height before a joint-space home move. This keeps the
        # gripper away from the upper arm in folded UR3 poses after placement.
        safe_pose = Pose()
        safe_pose.position.x = target_x
        safe_pose.position.y = target_y
        safe_pose.position.z = self.travel_height
        safe_pose.orientation = self.top_down_quaternion
        if not self._move_cartesian([safe_pose]) and not self._move_to_pose_target(safe_pose):
            self.node.get_logger().warn("Khong nang them duoc len do cao transfer; home se lap ke hoach lai")

        # 5. Cap nhat collision scene sau khi ngon tay da roi khoi cube.
        self.open_gripper()
        if not self.sync_collision_scene():
            return "SCENE_FAILED"
        return "SUCCESS"

    # =========================================================================
    # --- CAC SKILL NANG CAO (ADVANCED SKILLS) ---
    # =========================================================================

    def swap(self, object_a: str, object_b: str) -> str:
        """
        Skill nang cao: Hoan doi vi tri cua 2 vat the tren ban
        thong qua vung dem trung gian zone_temp.
        """
        self.node.get_logger().info(f"Thuc thi Skill nang cao: swap({object_a}, {object_b})")
        if object_a not in self.objects_config or object_b not in self.objects_config:
            return "INVALID_OBJECT"

        # Tim zone hien tai cua 2 vat (neu co)
        zone_of_a = None
        zone_of_b = None
        for z, occupant in self.zone_occupants.items():
            if occupant == object_a:
                zone_of_a = z
            elif occupant == object_b:
                zone_of_b = z

        buffer_zone = self.find_free_position()
        if buffer_zone is None:
            return "NO_FREE_POSITION"
        # 1. Chuyen A vao vung tam con trong
        if self.pick(object_a) != "SUCCESS":
            return "FAILED"
        if self.place(object_a, buffer_zone) != "SUCCESS":
            return "FAILED"

        # 2. Chuyen B vao vi tri cua A (hoac zone_a)
        target_b_zone = zone_of_a if zone_of_a else "zone_a"
        if self.pick(object_b) != "SUCCESS":
            return "FAILED"
        if self.place(object_b, target_b_zone) != "SUCCESS":
            return "FAILED"

        # 3. Chuyen A tu zone_temp vao vi tri cua B
        target_a_zone = zone_of_b if zone_of_b else "zone_b"
        if self.pick(object_a) != "SUCCESS":
            return "FAILED"
        if self.place(object_a, target_a_zone) != "SUCCESS":
            return "FAILED"

        self.home()
        return "SUCCESS"

    def stack(self, object_top: str, object_bottom: str) -> str:
        """
        Skill nang cao: Xep chong khoi hop object_top len tren dinh khoi hop object_bottom.
        Do cao dat = 0.06m (4cm khoi duoi + 2cm tam khoi tren).
        """
        self.node.get_logger().info(f"Thuc thi Skill nang cao: stack({object_top}, {object_bottom})")
        if object_top not in self.objects_config or object_bottom not in self.objects_config:
            return "INVALID_OBJECT"

        pos_bottom = self.object_positions.get(object_bottom)
        if not pos_bottom:
            return "INVALID_OBJECT"

        # 1. Gap khoi tren
        if self.pick(object_top) != "SUCCESS":
            return "FAILED"

        # 2. Di chuyen tren khong toi ngay tren dinh khoi duoi
        stack_approach = Pose()
        stack_approach.position.x = pos_bottom[0]
        stack_approach.position.y = pos_bottom[1]
        stack_approach.position.z = self.approach_height + 0.04
        stack_approach.orientation = self.top_down_quaternion

        if not self._move_cartesian([stack_approach]):
            if not self._move_to_pose_target(stack_approach):
                return "PLANNING_FAILED"

        # 3. Ha kẹp xuong do cao xep chong (z = 0.06m)
        stack_pose = Pose()
        stack_pose.position.x = pos_bottom[0]
        stack_pose.position.y = pos_bottom[1]
        stack_pose.position.z = 0.06
        stack_pose.orientation = self.top_down_quaternion

        if not self._move_cartesian([stack_pose]):
            if not self._move_to_pose_target(stack_pose):
                return "PLANNING_FAILED"

        # 4. Nha kep
        release_status = self.open_gripper()
        if release_status != "SUCCESS":
            return release_status
        final_pos = [pos_bottom[0], pos_bottom[1], 0.06]
        self.object_positions[object_top] = final_pos
        self._publish_dynamic_cube_state()

        # 5. Nhac kep len cao
        if not self._move_cartesian([stack_approach]):
            self._move_to_pose_target(stack_approach)

        return "SUCCESS"

    def _return_object_to_tray(self, object_name: str) -> str:
        """Gap 1 vat the ra khoi vung va hoan tra ve khay chua phoi ban dau."""
        if object_name not in self.objects_config:
            return "INVALID_OBJECT"

        init_pos = self.objects_config[object_name].get("initial_position", [0.24, 0.0, 0.02])

        # 1. Gap vat the
        pick_status = self.pick(object_name)
        if pick_status != "SUCCESS":
            return pick_status

        # 2. Tiep can tren khong cua khay chua ban dau
        tray_approach = Pose()
        tray_approach.position.x = init_pos[0]
        tray_approach.position.y = init_pos[1]
        tray_approach.position.z = self.approach_height
        tray_approach.orientation = self.top_down_quaternion

        if not self._move_cartesian([tray_approach]):
            if not self._move_to_pose_target(tray_approach):
                return "PLANNING_FAILED"

        # 3. Ha dat vat xuong khay
        tray_place = Pose()
        tray_place.position.x = init_pos[0]
        tray_place.position.y = init_pos[1]
        tray_place.position.z = self.place_height
        tray_place.orientation = self.top_down_quaternion

        if not self._move_cartesian([tray_place]):
            if not self._move_to_pose_target(tray_place):
                return "PLANNING_FAILED"

        # 4. Nha vat nhe nhang trong khay
        time.sleep(0.80)
        release_status = self.release_object_in_tray()
        if release_status != "SUCCESS":
            return release_status
        self.object_positions[object_name] = list(init_pos)
        self._publish_dynamic_cube_state()

        # 5. Nhac kep thang dung len tren khong (Retract)
        if not self._move_cartesian([tray_approach]):
            self._move_to_pose_target(tray_approach)

        # 6. Mo rong gripper va cap nhat PlanningScene
        self.open_gripper()
        if not self.sync_collision_scene():
            return "SCENE_FAILED"
        return "SUCCESS"

    def clear_zone(self, zone_name: str) -> str:
        occupant = self.zone_occupants.get(zone_name)
        if occupant is None:
            return "SUCCESS"
        destination = self.find_free_position()
        if destination is None or destination == zone_name:
            return "NO_FREE_POSITION"
        result = self.pick(occupant)
        if result != "SUCCESS":
            return result
        return self.place(occupant, destination)

    def clear_zones(self) -> str:
        """Gap tat ca cac vat the dang nam trong cac vung (Zone) tra ve khay chua ban dau."""
        self.node.get_logger().info("Thuc thi Skill: clear_zones() - Don sach cac vung truoc khi sap xep")
        occupied_zones = [z for z, occ in self.zone_occupants.items() if occ is not None]
        if not occupied_zones:
            self.node.get_logger().info("Tat ca cac vung deu dang trong. San sang sap xep!")
            return "SUCCESS"

        for z in occupied_zones:
            occ = self.zone_occupants.get(z)
            if occ:
                self.node.get_logger().info(f"[CLEAR] Gap '{occ}' ra khoi '{z}' ve khay ban dau...")
                res = self._return_object_to_tray(occ)
                if res != "SUCCESS":
                    return res
                self.zone_occupants[z] = None

        self.home()
        return "SUCCESS"

    def reset_scene(self) -> str:
        """Return cubes to source trays using pick and place motions only."""
        for name, data in self.objects_config.items():
            position = data["initial_position"]
            current = self.object_positions.get(name)
            if current and abs(current[0] - position[0]) < 0.015 and abs(current[1] - position[1]) < 0.015:
                continue
            result = self._return_object_to_tray(name)
            if result != "SUCCESS":
                return result
        return self.home()

    def get_cube_locations(self) -> dict:
        """Tra ve vi tri logic cua tung vat the ('zone_a', 'zone_b', 'zone_c', 'zone_temp' hoac 'source_tray')."""
        locations = {}
        for obj in self.objects_config:
            loc = "source_tray"
            for z, occupant in self.zone_occupants.items():
                if occupant == obj:
                    loc = z
                    break
            locations[obj] = loc
        return locations

    def get_scene_state(self) -> dict:
        """Bao cao trang thai logic day du cua Scene cho LLM Planner lap ke hoach toi uu."""
        return {
            "cube_locations": self.get_cube_locations(),
            "zone_occupants": dict(self.zone_occupants),
            "holding": self.holding_object,
            "positions": dict(self.object_positions),
            "camera_ready": bool(self.last_camera_state_at is not None and
                                 time.monotonic() - self.last_camera_state_at < 8.0 and
                                 self.camera_perception_state.get("camera_active") and
                                 len(self.camera_perception_state.get("detected_objects", {})) >=
                                 len(self.objects_config) - int(self.holding_object is not None))
        }

    def verify_placements(self, expected: dict, timeout_sec: float = 8.0) -> bool:
        """Confirm final cube locations using new camera images after the arm homes."""
        started_at = time.monotonic()
        deadline = started_at + timeout_sec
        while time.monotonic() < deadline:
            state = self.camera_perception_state
            detected = state.get("detected_objects", {})
            if (self.last_camera_state_at is not None and
                    self.last_camera_state_at > started_at and
                    time.monotonic() - self.last_camera_state_at < 8.0 and
                    state.get("camera_active") and
                    all(detected.get(obj, {}).get("location") == zone
                        for obj, zone in expected.items())):
                return True
            time.sleep(0.1)
        self.node.get_logger().error(f"Camera khong xac nhan duoc vi tri dich: {expected}")
        return False

    def inspect_scene(self) -> dict:
        """Bao cao vi tri tat ca cac vat the, trang thai zone va trang thai tay kep."""
        return self.get_scene_state()

    def detect_objects(self) -> dict:
        """Kiem tra va nhan dien vi tri cac khoi hop qua camera perception."""
        self.node.get_logger().info("Thuc thi Skill: detect_objects() qua Camera")
        detected = self.camera_perception_state.get("detected_objects", {})
        if not detected:
            return self.get_cube_locations()
        result = {}
        for obj, info in detected.items():
            result[obj] = info.get("location", "unknown")
        return result

    def check_zone(self, zone_name: str) -> str:
        """Kiem tra trang thai cua 1 zone cu the (co vat chiem cho hay trong)."""
        self.node.get_logger().info(f"Thuc thi Skill: check_zone({zone_name})")
        return self.zone_occupants.get(zone_name)

    def find_free_position(self) -> str:
        """Tim mot vi tri vung dem hoac vi tri trong phu hop tren ban."""
        self.node.get_logger().info("Thuc thi Skill: find_free_position()")
        temp_candidates = ["zone_temp_3", "zone_temp_2", "zone_temp_1",
                           "zone_a", "zone_b", "zone_c"]
        for z in temp_candidates:
            if z in self.zones_config and self.zone_occupants.get(z) is None:
                return z
        return None

    # =========================================================================
    # --- CAC HAM BO TRO MOVEIT 2 ---
    # =========================================================================

    def _get_target_xy(self, name: str):
        """Lay toa do (X, Y) cua object hoac zone."""
        if name in self.object_positions:
            return self.object_positions[name]
        if name in self.zones_config:
            return self.zones_config[name].get("position")[:2]
        return None

    def _wait_for_future(self, future, timeout_sec: float = 10.0) -> bool:
        """Cho future hoan thanh ma khong gay deadlock hoac xung dot spin."""
        start_time = time.time()
        while not future.done() and (time.time() - start_time < timeout_sec):
            time.sleep(0.015)
        return future.done()

    def _move_to_joint_target(self, joint_values: list) -> bool:
        """Lap ke hoach va thuc thi dich khop toi cac goc hop le da cau hinh."""
        if not self._move_group_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("MoveGroup action server khong phan hoi!")
            return False

        unwrapped_targets = self._unwrap_joint_target(joint_values)

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        goal_msg.request.pipeline_id = "ompl"
        goal_msg.request.planner_id = "RRTConnectkConfigDefault"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 8.0
        goal_msg.request.max_velocity_scaling_factor = 0.55
        goal_msg.request.max_acceleration_scaling_factor = 0.30
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.request.start_state.is_diff = True

        constraints = Constraints()
        for idx, (name, val) in enumerate(zip(self.joint_names, unwrapped_targets)):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(val)
            jc.tolerance_above = 0.05
            jc.tolerance_below = 0.05
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal_msg.request.goal_constraints.append(constraints)

        send_goal_future = self._move_group_client.send_goal_async(goal_msg)
        self._wait_for_future(send_goal_future, timeout_sec=10.0)

        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.node.get_logger().warn("Joint goal bi tu choi hoac timeout.")
            return False

        get_result_future = goal_handle.get_result_async()
        self._wait_for_future(get_result_future, timeout_sec=120.0)

        result = get_result_future.result()
        return bool(result and result.result.error_code.val == 1)

    def _move_to_pose_target(self, target_pose: Pose) -> bool:
        """Lap ke hoach va thuc thi vi tri khong gian Descartes Pose voi rang buoc huong nghiem ngat."""
        if not self._move_group_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("MoveGroup action server khong phan hoi!")
            return False

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        goal_msg.request.pipeline_id = "ompl"
        goal_msg.request.planner_id = "RRTConnectkConfigDefault"
        goal_msg.request.num_planning_attempts = 8
        goal_msg.request.allowed_planning_time = 4.0
        goal_msg.request.max_velocity_scaling_factor = 0.55
        goal_msg.request.max_acceleration_scaling_factor = 0.30
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.request.start_state.is_diff = True

        # Dinh vi Constraint cho end-effector tool0
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.header.stamp = self.node.get_clock().now().to_msg()
        # Offset tool0 do co gan gripper (gripper dai ~8.2cm tu tool0 den tam dem silicone)
        pose_stamped.pose = Pose()
        pose_stamped.pose.position.x = target_pose.position.x
        pose_stamped.pose.position.y = target_pose.position.y
        pose_stamped.pose.position.z = target_pose.position.z + 0.082
        pose_stamped.pose.orientation = target_pose.orientation

        # Rang buoc vi tri
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "base_link"
        pos_constraint.link_name = "tool0"
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0

        bv = BoundingVolume()
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.006]
        bv.primitives.append(sp)
        bv.primitive_poses.append(pose_stamped.pose)
        pos_constraint.constraint_region = bv
        pos_constraint.weight = 1.0

        # Rang buoc huong kẹp: KHONG CHO PHEP quay loan xa quanh truc Z
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = "base_link"
        orient_constraint.link_name = "tool0"
        orient_constraint.orientation = target_pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.10
        orient_constraint.absolute_y_axis_tolerance = 0.10
        orient_constraint.absolute_z_axis_tolerance = 0.20
        orient_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(orient_constraint)

        goal_msg.request.goal_constraints.append(constraints)

        send_goal_future = self._move_group_client.send_goal_async(goal_msg)
        self._wait_for_future(send_goal_future, timeout_sec=10.0)

        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            return False

        get_result_future = goal_handle.get_result_async()
        self._wait_for_future(get_result_future, timeout_sec=120.0)

        result = get_result_future.result()
        if result and result.result.error_code.val == 1:
            return True
        # A position controller can report GOAL_TOLERANCE_VIOLATED on a wrist
        # joint even when the measured TCP has reached the checked pose path.
        # Accept only CONTROL_FAILED (-4), never a failed collision/planning check.
        if result and result.result.error_code.val == -4 and self._tool_reached(target_pose, 2.0):
            self.node.get_logger().warn("Controller bao sai so khop, nhung TF xac nhan TCP da toi dich")
            return True
        return False

    def _move_cartesian(self, waypoints: list) -> bool:
        """Di chuyen duong thang Descartes mem mai, khong giat lag, toc do nhanh, tinh toan timestamp chuan."""
        if not self._cartesian_path_client.wait_for_service(timeout_sec=2.0):
            return False

        req = GetCartesianPath.Request()
        req.header.frame_id = "base_link"
        req.header.stamp = self.node.get_clock().now().to_msg()
        req.group_name = "ur_manipulator"
        req.link_name = "tool0"
        req.start_state.is_diff = True

        # Offset z cho tool0 do co gripper
        offset_wps = []
        for wp in waypoints:
            p = Pose()
            p.position.x = wp.position.x
            p.position.y = wp.position.y
            p.position.z = wp.position.z + 0.082
            p.orientation = wp.orientation
            offset_wps.append(p)

        req.waypoints = offset_wps
        req.max_step = 0.008
        req.jump_threshold = 1.5
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = min(0.55, max(0.1, self.cartesian_max_vel))
        req.max_acceleration_scaling_factor = 0.25

        future = self._cartesian_path_client.call_async(req)
        self._wait_for_future(future, timeout_sec=5.0)

        res = future.result()
        if not res or res.fraction < 0.995 or res.error_code.val != 1:
            return False

        traj = res.solution.joint_trajectory
        if not traj.points:
            return False
        if any(any(abs(b - a) > 0.35 for a, b in zip(previous.positions, current.positions))
               for previous, current in zip(traj.points, traj.points[1:])):
            self.node.get_logger().warn("Cartesian path co buoc nhay khop >0.35 rad, chuyen sang MoveGroup")
            return False
        times = [pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
                 for pt in traj.points]
        if len(times) > 1 and any(b <= a for a, b in zip(times, times[1:])):
            self.node.get_logger().warn("Cartesian trajectory khong duoc time-parameterize")
            return False

        # Thuc thi trajectory bang MoveIt ExecuteTrajectory (yeu cau RobotTrajectory)
        if self._execute_traj_client.wait_for_server(timeout_sec=3.0):
            goal = ExecuteTrajectory.Goal()
            goal.trajectory = res.solution
            exec_future = self._execute_traj_client.send_goal_async(goal)
            self._wait_for_future(exec_future, timeout_sec=10.0)
            handle = exec_future.result()
            if handle and handle.accepted:
                res_future = handle.get_result_async()
                self._wait_for_future(res_future, timeout_sec=120.0)
                exec_res = res_future.result()
                if exec_res:
                    err = getattr(getattr(exec_res, "result", None), "error_code", None)
                    err_val = getattr(err, "val", 0) if err else 0
                    status = getattr(exec_res, "status", 0)
                    if err_val == 1 and status == 4:
                        return True
                    if err_val == -4 and waypoints and self._tool_reached(waypoints[-1], 2.0):
                        self.node.get_logger().warn(
                            "Cartesian controller bao sai so khop, nhung TCP da toi dich"
                        )
                        return True
        return False

    def sync_collision_scene(self, exclude: str = None) -> bool:
        """Mirror camera-derived cube boxes into the MoveIt planning scene."""
        if not self._planning_scene_client.wait_for_service(timeout_sec=2.0):
            return False
        scene = PlanningScene()
        scene.is_diff = True
        for name, config in self.objects_config.items():
            collision = CollisionObject()
            collision.header.frame_id = "base_link"
            collision.id = name
            if name in (exclude, self.holding_object):
                collision.operation = CollisionObject.REMOVE
            else:
                collision.operation = CollisionObject.ADD
                box = SolidPrimitive()
                box.type = SolidPrimitive.BOX
                box.dimensions = [float(x) for x in config["size"]]
                collision.primitives.append(box)
                pose = Pose()
                position = self.object_positions[name]
                pose.position.x, pose.position.y, pose.position.z = map(float, position)
                pose.orientation.w = 1.0
                collision.primitive_poses.append(pose)
            scene.world.collision_objects.append(collision)
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self._planning_scene_client.call_async(request)
        return self._wait_for_future(future, timeout_sec=3.0) and bool(future.result().success)

    def _attach_object_to_robot(self, object_name: str) -> bool:
        if not self._planning_scene_client.wait_for_service(timeout_sec=2.0):
            return False
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        attached = AttachedCollisionObject()
        attached.link_name = "tool0"
        attached.object.header.frame_id = "tool0"
        attached.object.id = object_name
        attached.object.operation = CollisionObject.ADD
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [float(x) for x in self.objects_config[object_name]["size"]]
        attached.object.primitives.append(box)
        pose = Pose()
        pose.position.z = 0.086
        pose.orientation.w = 1.0
        attached.object.primitive_poses.append(pose)
        attached.touch_links = ["tool0", "gripper_flange_link", "gripper_base_link",
                                "gripper_left_finger", "gripper_right_finger"]
        scene.robot_state.attached_collision_objects.append(attached)
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self._planning_scene_client.call_async(request)
        return self._wait_for_future(future, timeout_sec=3.0) and bool(future.result().success)

    def _detach_object_from_robot(self, object_name: str) -> bool:
        if not self._planning_scene_client.wait_for_service(timeout_sec=2.0):
            return False
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        attached = AttachedCollisionObject()
        attached.link_name = "tool0"
        attached.object.id = object_name
        attached.object.operation = CollisionObject.REMOVE
        scene.robot_state.attached_collision_objects.append(attached)
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self._planning_scene_client.call_async(request)
        return self._wait_for_future(future, timeout_sec=3.0) and bool(future.result().success)
