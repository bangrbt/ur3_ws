#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Skills Library (phien ban da sua loi chuyen dong + loi kep truot):
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
- swap(object_a, object_b): Hoan doi 2 vat the su dung vung dem trong
- stack(object_top, object_bottom): Xep chong vat the
- reset_scene(): Dua tat ca cac khoi hop ve lai 5 khay phoi ban dau
- inspect_scene(): Tra cuu trang thai toan bo khong gian lam viec

Moi skill tra ve trang thai thuc thi: SUCCESS, FAILED, INVALID_OBJECT, PLANNING_FAILED, ...

Cac tham so tuy chon trong scene_config["motion_params"]:
- tcp_offset (0.078)            : khoang cach tool0 -> tam hai pad theo URDF
- reach_tolerance (0.012)       : sai so XY/Z toi da (m) truoc khi dong kep
- cartesian_fraction (0.98)     : ti le toi thieu cua duong Cartesian
- cartesian_avoid_collisions    : True/False, tranh va cham khi Cartesian
- max_joint_travel (3.5)        : tong goc quay toi da (rad) cua 1 khop trong 1 chuyen dong
- require_camera (False)        : True -> tu choi pick neu camera chua san sang
- grasp_height (0.029)          : tam pad cao hon tam cube 9 mm, khong quet khay
- use_gazebo_attach (True)      : True -> dung DetachableJoint cua Gazebo de giu khoi chac chan
- grip_close_pos (0.007)        : nen moi pad 1 mm vao cube 4 cm
- carry_vel_scale (0.45)        : he so toc do khi dang cam vat
- carry_acc_scale (0.30)        : he so gia toc khi dang cam vat
"""

import time
import math
import json
import re
import subprocess
import threading
from contextlib import contextmanager

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64
from ros_gz_interfaces.msg import Contacts
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
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
    """Thu vien ky nang MoveIt 2 cho canh tay UR3: quy dao on dinh, khong lat khop, chong de vat the."""

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
        # Offset world cua tam cube so voi TCP tai thoi diem kep. Offset nay chi
        # duoc ghi sau khi hai cam bien ngon tay cung xac nhan tiep xuc.
        self.grasp_offsets = {}
        self._finger_contacts = {"left": set(), "right": set()}
        self._finger_contact_at = {"left": 0.0, "right": 0.0}
        self._finger_contact_positions = {"left": {}, "right": {}}
        self._contact_stream_ever_seen = False

        # Camera chi duoc ghi de toa do khi tay may KHONG chuyen dong va khong che camera
        self._busy_count = 0
        self._arm_clear_of_view = True  # gia dinh luc khoi dong tay may dang o home

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
        self.left_contact_sub = self.node.create_subscription(
            Contacts, "/gripper/left_contacts", self._left_contact_cb, 10
        )
        self.right_contact_sub = self.node.create_subscription(
            Contacts, "/gripper/right_contacts", self._right_contact_cb, 10
        )

        # Gripper Joint State Publisher (phuc vu hien thi co bop muot ma lien tuc tren RViz)
        self.gripper_joint_pub = self.node.create_publisher(JointState, "/joint_states", 10)
        self.current_gripper_pos = 0.018
        self.target_gripper_pos = 0.018
        # Co bao dang animate: timer khong duoc noi suy chen vao luc nay (tranh gianh co)
        self._animating = False
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

        # Orientation chu vi thang dung vuong goc mat ban (Top-down grasp)
        self.top_down_quaternion = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        # Quaternion thao tac duoc chuan hoa ve roll=pi, pitch=0; chi giu yaw
        # cua posture hien tai de khong xoay wrist_3 thua.
        self.operation_quaternion = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)

        # Tham so chieu cao / hinh hoc
        self.approach_height = float(self.motion_params.get("approach_height", 0.13))
        self.travel_height = float(self.motion_params.get("travel_height", 0.14))
        self.cube_rest_height = float(self.motion_params.get("cube_rest_height", 0.020))
        # Pad nam cao hon tam cube 9 mm: van om phan lon than cube, con dau
        # ngon cach mat ban de sai so roll/pitch nho khong the lam cham khay.
        self.grasp_height = float(self.motion_params.get("grasp_height", 0.029))
        self.place_height = float(self.motion_params.get("place_height", 0.029))
        self.grasp_center_offset = self.grasp_height - self.cube_rest_height
        self.home_joints = list(self.motion_params.get("home_joints", [-1.57, -1.3, 1.5, -1.7, -1.57, 0.0]))
        # Offset tool0 -> tam dem kep: gom ve MOT hang so duy nhat
        self.tcp_offset = float(self.motion_params.get("tcp_offset", 0.078))

        # Tham so an toan / do chinh xac
        self.reach_tolerance = float(self.motion_params.get("reach_tolerance", 0.006))
        self.cartesian_fraction = float(self.motion_params.get("cartesian_fraction", 0.98))
        self.cartesian_avoid_collisions = bool(self.motion_params.get("cartesian_avoid_collisions", True))
        self.max_joint_travel = float(self.motion_params.get("max_joint_travel", 3.5))
        self.require_camera = bool(self.motion_params.get("require_camera", False))
        self.cartesian_max_vel = float(self.motion_params.get("cartesian_speed", 0.65))
        self.cartesian_max_acc = float(self.motion_params.get("cartesian_acceleration", 0.45))
        self.approach_vel_scale = float(self.motion_params.get("approach_speed", 0.18))
        self.approach_acc_scale = float(self.motion_params.get("approach_acceleration", 0.12))
        self.lift_vel_scale = float(self.motion_params.get("lift_speed", 0.35))
        self.lift_acc_scale = float(self.motion_params.get("lift_acceleration", 0.25))

        # Tham so kep / giu vat
        self.use_gazebo_attach = bool(self.motion_params.get("use_gazebo_attach", True))
        self.grip_open_pos = float(self.motion_params.get("grip_open_pos", 0.018))
        self.grip_close_pos = float(self.motion_params.get("grip_close_pos", 0.007))
        self.grip_hold_pos = float(self.motion_params.get("grip_hold_pos", 0.0060))
        self.carry_vel_scale = float(self.motion_params.get("carry_vel_scale", 0.45))
        self.carry_acc_scale = float(self.motion_params.get("carry_acc_scale", 0.30))
        self.grasp_xy_tolerance = float(self.motion_params.get("grasp_xy_tolerance", 0.008))
        self.grasp_z_tolerance = float(self.motion_params.get("grasp_z_tolerance", 0.006))
        self.require_bilateral_contact = bool(
            self.motion_params.get("require_bilateral_contact", True)
        )

        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        # Timer dinh ky theo doi tool0 khi dang gap vat de cap nhat vi tri vat the
        self.tracking_timer = self.node.create_timer(0.1, self._tracking_callback)

        # Khoi tao gripper; Gazebo joint duoc tao chi tai luc gap cube
        self._init_gripper_hardware()

    # =========================================================================
    # --- TRANG THAI BAN / DONG BO CAMERA ---
    # =========================================================================

    @contextmanager
    def _busy(self):
        """Danh dau tay may dang chuyen dong: camera khong duoc ghi de toa do vat."""
        self._busy_count += 1
        self._arm_clear_of_view = False
        try:
            yield
        finally:
            self._busy_count -= 1

    def _camera_updates_allowed(self) -> bool:
        return self._busy_count == 0 and self._arm_clear_of_view

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
            self.target_gripper_pos = self.grip_open_pos
            self.current_gripper_pos = self.grip_open_pos
            self._send_gripper_joint_cmd(self.grip_open_pos, -self.grip_open_pos)
        threading.Thread(target=_async_init, daemon=True).start()

    def _camera_state_cb(self, msg: String):
        """Chi nhan quan sat day du, moi, va CHI khi tay may khong chuyen dong / khong che camera."""
        try:
            data = json.loads(msg.data)
            self.camera_perception_state = data
            self.last_camera_state_at = time.monotonic()
            if not self._camera_updates_allowed():
                return
            detected = data.get("detected_objects", {})
            expected = len(self.objects_config) - int(self.holding_object is not None)
            if not data.get("camera_active") or len(detected) < expected:
                return
            for obj_name, info in detected.items():
                if self.holding_object != obj_name and "x" in info and "y" in info:
                    self.object_positions[obj_name] = [
                        info["x"], info["y"], self.cube_rest_height
                    ]
            zones = data.get("zone_occupants", {})
            for zone, occupant in zones.items():
                if zone in self.zone_occupants and occupant != self.holding_object:
                    self.zone_occupants[zone] = occupant
            self._publish_dynamic_cube_state()
        except (ValueError, TypeError, KeyError) as exc:
            self.node.get_logger().warn(f"Camera state khong hop le: {exc}")

    def _contact_snapshot(self, msg: Contacts):
        """Lay ten cube va trung binh diem tiep xuc vat ly trong world."""
        touched = set()
        samples = {}
        for contact in msg.contacts:
            pair = f"{contact.collision1.name} {contact.collision2.name}"
            for object_name in self.objects_config:
                if object_name in pair:
                    touched.add(object_name)
                    samples.setdefault(object_name, []).extend(contact.positions)
        centers = {}
        for object_name, positions in samples.items():
            if positions:
                count = float(len(positions))
                centers[object_name] = (
                    sum(p.x for p in positions) / count,
                    sum(p.y for p in positions) / count,
                    sum(p.z for p in positions) / count,
                )
        return touched, centers

    def _left_contact_cb(self, msg: Contacts):
        self._contact_stream_ever_seen = True
        touched, centers = self._contact_snapshot(msg)
        self._finger_contacts["left"] = touched
        self._finger_contact_positions["left"] = centers
        self._finger_contact_at["left"] = time.monotonic()

    def _right_contact_cb(self, msg: Contacts):
        self._contact_stream_ever_seen = True
        touched, centers = self._contact_snapshot(msg)
        self._finger_contacts["right"] = touched
        self._finger_contact_positions["right"] = centers
        self._finger_contact_at["right"] = time.monotonic()

    def _record_grasp_from_contacts(self, object_name: str):
        """Lay trung diem hai mat tiep xuc lam tam cube sau khi da ep can."""
        left = self._finger_contact_positions["left"].get(object_name)
        right = self._finger_contact_positions["right"].get(object_name)
        if left is None or right is None:
            return
        contact_midpoint = tuple((a + b) * 0.5 for a, b in zip(left, right))
        # Trung diem hai mat tiep xuc cho tam XY rat tot, nhung Z cua cac diem
        # tiep xuc phu thuoc solver va khong phai tam khoi. Giu Z da biet tu
        # camera/trang thai vat ly de cube khong bi ve treo len trong RViz.
        known_z = float(
            self.object_positions.get(object_name, [0.0, 0.0, self.cube_rest_height])[2]
        )
        cube_center = (contact_midpoint[0], contact_midpoint[1], known_z)
        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link", "tool0", rclpy.time.Time()
            ).transform
            tcp = (
                transform.translation.x,
                transform.translation.y,
                transform.translation.z - self.tcp_offset,
            )
            self.object_positions[object_name] = list(cube_center)
            self.grasp_offsets[object_name] = tuple(
                cube_center[index] - tcp[index] for index in range(3)
            )
        except Exception:
            pass

    def _fresh_contact_sides(self, object_name: str):
        now = time.monotonic()
        left_ok = (
            object_name in self._finger_contacts["left"] and
            now - self._finger_contact_at["left"] <= 0.50
        )
        right_ok = (
            object_name in self._finger_contacts["right"] and
            now - self._finger_contact_at["right"] <= 0.50
        )
        return left_ok, right_ok

    def _bilateral_contact(self, object_name: str, timeout_sec: float = 1.0,
                           log_failure: bool = True) -> bool:
        """Chi xac nhan grasp khi ca hai pad cung cham dung cube trong Gazebo."""
        if not self.require_bilateral_contact:
            return True
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            left_ok, right_ok = self._fresh_contact_sides(object_name)
            if left_ok and right_ok:
                return True
            time.sleep(0.02)
        if log_failure:
            self.node.get_logger().error(
                f"Grasp bi tu choi sau khi da ep va can tam: "
                f"{object_name} (left={self._finger_contacts['left']}, "
                f"right={self._finger_contacts['right']})"
            )
        return False

    def _gazebo_entity_position(self, entity_name: str):
        """Doc pose that cua model tu Gazebo SceneBroadcaster.

        Ham nay chi dung de hau kiem grasp khi ban Gazebo hien tai khong phat
        contact sensor. Khong dat pose va khong di chuyen object.
        """
        try:
            result = subprocess.run(
                ["ign", "topic", "-e", "-t", "/world/default/pose/info",
                 "-n", "1"],
                capture_output=True, text=True, timeout=2.0,
            )
            pattern = (
                r'pose\s*\{\s*name:\s*"' + re.escape(entity_name) +
                r'".*?position\s*\{\s*x:\s*([-+0-9.eE]+)\s*'
                r'y:\s*([-+0-9.eE]+)\s*z:\s*([-+0-9.eE]+)'
            )
            match = re.search(pattern, result.stdout, re.DOTALL)
            if match:
                return tuple(float(match.group(index)) for index in range(1, 4))
        except (OSError, subprocess.TimeoutExpired, ValueError):
            pass
        return None

    def _confirm_enclosed_grasp(self, object_name: str) -> bool:
        """Xac nhan cube nam trong khe kep sau khi hai ngon da ep vat ly.

        Dung khi contact transport cua Gazebo Fortress im lang. Dieu kien gom:
        TCP dung tam, pad da dong nho hon be rong cube, va pose Gazebo (neu doc
        duoc) van nam giua hai ngon. Day la hau kiem hinh hoc sau khi ep, khong
        teleport va khong tu dong dinh mot vat o xa gripper.
        """
        if not self._verify_grasp_alignment(object_name):
            return False

        cube_size = self.objects_config.get(object_name, {}).get(
            "size", [0.04, 0.04, 0.04]
        )
        cube_width = float(max(cube_size[0], cube_size[1]))
        # Hinh hoc URDF: mat trong hai pad cach nhau 24 mm + 2*q.
        commanded_gap = 0.024 + 2.0 * float(self.current_gripper_pos)
        if commanded_gap > cube_width - 0.001:
            self.node.get_logger().error(
                f"Khe kep {commanded_gap * 1000:.1f} mm chua ep vao cube "
                f"{cube_width * 1000:.1f} mm"
            )
            return False

        gazebo_position = self._gazebo_entity_position(object_name)
        if gazebo_position is not None:
            self.object_positions[object_name] = list(gazebo_position)
            if not self._verify_grasp_alignment(object_name, log_failure=False):
                self.node.get_logger().warn(
                    f"{object_name} bi xe dich khi dong ngon; mo nhe, "
                    "can tam theo pose Gazebo va kep lai"
                )
                return self._recenter_from_gazebo_pose(object_name)

        self.node.get_logger().warn(
            f"Gazebo khong phat contact stream; da xac nhan {object_name} "
            f"nam trong khe {commanded_gap * 1000:.1f} mm va GIU LUC KEP"
        )
        return True

    def _recenter_from_gazebo_pose(self, object_name: str,
                                   max_attempts: int = 2) -> bool:
        """Can tam lai grasp khi cube bi day nhe trong luc kep.

        Cube chi duoc doc pose. Robot mo khe co clearance, MoveL ngang vai mm
        toi tam moi, sau do ep lai. Khong set pose / teleport object.
        """
        cube_size = self.objects_config.get(object_name, {}).get(
            "size", [0.04, 0.04, 0.04]
        )
        cube_width = float(max(cube_size[0], cube_size[1]))
        # Khe trong = 24 mm + 2*q. Them 6 mm clearance tong de hai pad
        # khong keo cube khi can tam ngang.
        recenter_open = max(
            0.011, (cube_width + 0.006 - 0.024) * 0.5
        )

        for attempt in range(1, max_attempts + 1):
            position = self._gazebo_entity_position(object_name)
            if position is None:
                position = tuple(self.object_positions[object_name][:3])
            obj_x, obj_y, obj_z = map(float, position)

            try:
                transform = self.tf_buffer.lookup_transform(
                    "base_link", "tool0", rclpy.time.Time()
                ).transform
                error = math.hypot(
                    transform.translation.x - obj_x,
                    transform.translation.y - obj_y,
                )
            except Exception:
                return False

            # Sai lech qua lon nghia la cube khong con o trong vung kep; khong
            # duoc gan vat tu xa vao gripper.
            if error > 0.018:
                self.node.get_logger().error(
                    f"Khong the can tam {object_name}: lech "
                    f"{error * 1000:.1f} mm > 18 mm"
                )
                return False

            self.node.get_logger().info(
                f"Can tam grasp {object_name} lan {attempt}: "
                f"dich TCP {error * 1000:.1f} mm"
            )
            self._animate_gripper(recenter_open, steps=5, duration=0.15)

            corrected_pose = self._top_down_pose(
                obj_x, obj_y, obj_z + self.grasp_center_offset
            )
            if not self._move_cartesian(
                    [corrected_pose], min_fraction=0.99,
                    vel_scale=0.08, acc_scale=0.06):
                return False
            if not self._tool_reached(corrected_pose, timeout_sec=2.0):
                return False

            # Dong lai tu ngoai vao trong, tranh lenh buoc lon day cube lan nua.
            for target in (0.009, self.grip_close_pos, 0.006, 0.005):
                self._animate_gripper(target, steps=4, duration=0.12)
            time.sleep(0.12)

            updated = self._gazebo_entity_position(object_name)
            if updated is not None:
                self.object_positions[object_name] = list(updated)
            if self._verify_grasp_alignment(
                    object_name, log_failure=(attempt == max_attempts)):
                self.node.get_logger().info(
                    f"Da can tam va kep lai {object_name} thanh cong"
                )
                return True

        return False

    def _recenter_from_single_contact(self, object_name: str) -> bool:
        """Neu mot pad cham truoc, mo nhe va dich TCP 2 mm ve phia cube."""
        left_ok, right_ok = self._fresh_contact_sides(object_name)
        if left_ok == right_ok:
            return False

        # Ghi lai phia cham truoc khi mo, vi message contact se mat ngay sau do.
        direction = 1.0 if left_ok else -1.0
        self._animate_gripper(max(0.010, self.grip_close_pos + 0.003),
                              steps=4, duration=0.10)
        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link", "tool0", rclpy.time.Time()
            ).transform
            q = transform.rotation
            # Cot Y cua ma tran quay: huong tu TCP toi ngon trai trong world.
            local_y_x = 2.0 * (q.x * q.y - q.z * q.w)
            local_y_y = 1.0 - 2.0 * (q.x * q.x + q.z * q.z)
            correction = 0.002
            tcp_z = transform.translation.z - self.tcp_offset
            corrected_pose = self._top_down_pose(
                transform.translation.x + direction * correction * local_y_x,
                transform.translation.y + direction * correction * local_y_y,
                tcp_z,
            )
        except Exception as exc:
            self.node.get_logger().warn(f"Khong tinh duoc buoc can tam grasp: {exc}")
            return False

        side = "trai" if left_ok else "phai"
        self.node.get_logger().info(
            f"Chi pad {side} cham {object_name}; can tam TCP 2 mm roi kep lai"
        )
        if not self._move_cartesian(
                [corrected_pose], min_fraction=0.99,
                vel_scale=0.10, acc_scale=0.08):
            return False
        return self._verify_grasp_alignment(object_name)

    def _adaptive_gripper_close(self, object_name: str) -> bool:
        """Dong dan theo contact, can tam neu cham mot ben, roi tao preload nhe."""
        stages = [0.010, 0.0085, self.grip_close_pos, 0.006, 0.005]
        # Loai gia tri trung va dam bao chi dong vao trong theo thu tu giam dan.
        stages = sorted({max(0.0045, float(value)) for value in stages}, reverse=True)
        recentered = False

        for target in stages:
            self._animate_gripper(target, steps=4, duration=0.12)
            if self._bilateral_contact(object_name, 0.22, log_failure=False):
                # Them 0.8 mm preload moi ben de tao ma sat, khong ep sau nhu q=5 mm.
                preload = max(0.0055, target - 0.0008)
                if preload < target:
                    self._animate_gripper(preload, steps=3, duration=0.09)
                stable = self._bilateral_contact(
                    object_name, 0.30, log_failure=False
                )
                if stable:
                    self._record_grasp_from_contacts(object_name)
                return stable

            left_ok, right_ok = self._fresh_contact_sides(object_name)
            if left_ok != right_ok and not recentered:
                if self._recenter_from_single_contact(object_name):
                    recentered = True
                    self._finger_contacts = {"left": set(), "right": set()}
                    self._finger_contact_at = {"left": 0.0, "right": 0.0}
                    self._finger_contact_positions = {"left": {}, "right": {}}
                    continue

        if self._bilateral_contact(object_name, 0.35, log_failure=False):
            self._record_grasp_from_contacts(object_name)
            return True

        # Phan biet ro "cam bien da tra ve nhung grasp mot ben" voi "Gazebo
        # khong co contact stream". Truong hop thu hai khong duoc mo kep va lam
        # roi cube sau khi hai pad da bao quanh vat.
        if not self._contact_stream_ever_seen:
            return self._confirm_enclosed_grasp(object_name)

        self.node.get_logger().error(
            f"Hai contact sensor co du lieu nhung chua cung cham {object_name}: "
            f"left={self._finger_contacts['left']}, "
            f"right={self._finger_contacts['right']}"
        )
        return False

    def _joint_state_cb(self, msg: JointState):
        """Cap nhat gia tri khop hien tai."""
        for name, pos in zip(msg.name, msg.position):
            self.current_joint_positions[name] = pos

    def _tracking_callback(self):
        """Cap nhat marker theo TCP va offset grasp da do, khong ep cube vao tam kep."""
        if self.holding_object:
            try:
                t = self.tf_buffer.lookup_transform("base_link", "tool0", rclpy.time.Time())
                tx = t.transform.translation.x
                ty = t.transform.translation.y
                tz = max(self.cube_rest_height,
                         t.transform.translation.z - self.tcp_offset)
                dx, dy, dz = self.grasp_offsets.get(
                    self.holding_object,
                    (0.0, 0.0, -self.grasp_center_offset),
                )
                self.object_positions[self.holding_object] = [
                    tx + dx, ty + dy, max(self.cube_rest_height, tz + dz)
                ]
                self._publish_dynamic_cube_state()
            except Exception:
                pass

    def _verify_grasp_alignment(self, object_name: str,
                                log_failure: bool = True) -> bool:
        """Kiem tra TCP dang bao quanh tam cube truoc khi cho phep kep/attach."""
        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link", "tool0", rclpy.time.Time()
            ).transform
            tcp_x = float(transform.translation.x)
            tcp_y = float(transform.translation.y)
            tcp_z = float(transform.translation.z - self.tcp_offset)
            obj_x, obj_y, obj_z = map(float, self.object_positions[object_name][:3])
        except Exception as exc:
            self.node.get_logger().error(f"Khong kiem tra duoc tam grasp: {exc}")
            return False

        xy_error = math.hypot(tcp_x - obj_x, tcp_y - obj_y)
        # TCP/pad duoc dat cao hon tam cube co chu y de dau ngon khong cham
        # mat khay. Kiem tra quanh chieu cao mong muon, khong quanh tam cube.
        expected_tcp_z = obj_z + self.grasp_center_offset
        z_error = abs(tcp_z - expected_tcp_z)
        if xy_error > self.grasp_xy_tolerance or z_error > self.grasp_z_tolerance:
            if not log_failure:
                return False
            self.node.get_logger().error(
                f"Grasp lech tam: XY={xy_error * 1000:.1f} mm, "
                f"Z={z_error * 1000:.1f} mm (TCP phai cao hon tam cube "
                f"{self.grasp_center_offset * 1000:.1f} mm)"
            )
            return False

        self.grasp_offsets[object_name] = (
            obj_x - tcp_x, obj_y - tcp_y, obj_z - tcp_z
        )
        return True

    def _publish_dynamic_cube_state(self):
        """Phat trang thai vi tri cac hop len topic de SceneSpawner cap nhat RViz."""
        try:
            msg = String()
            msg.data = json.dumps(self.object_positions)
            self.cube_state_pub.publish(msg)
        except Exception:
            pass

    def _unwrap_joint_target(self, goal_joints: list) -> list:
        """Giu nguyen goc khop cau hinh (khong modulo 2*pi) de nam trong gioi han URDF."""
        return [float(value) for value in goal_joints]

    # =========================================================================
    # --- CAC SKILL NGUYEN THUY ---
    # =========================================================================

    def home(self) -> str:
        """Dua tay may ve tu the home hop le; MoveIt chon duong di tranh va cham."""
        self.node.get_logger().info("Thuc thi Skill: home()")
        with self._busy():
            for attempt in range(3):
                if self._move_to_joint_target(self.home_joints):
                    self._arm_clear_of_view = True  # tay may da roi khoi tam nhin camera
                    return "SUCCESS"
                self.node.get_logger().warn(f"Home planning attempt {attempt + 1}/3 failed; retrying")
                time.sleep(0.25)
        return "PLANNING_FAILED"

    def _publish_gripper_state(self):
        """Noi suy vi tri ngon tay va publish lien tuc tren RViz (/joint_states) va Gazebo.

        Khi _animate_gripper dang chay (self._animating) thi KHONG noi suy/gui lenh o day
        de tranh gianh co voi vong lap animate."""
        step = 0.0025
        if not self._animating and abs(self.current_gripper_pos - self.target_gripper_pos) > 1e-4:
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
        """Send gradual position commands so Gazebo fingers contact the cube gently.

        - Gan target TRUOC khi animate de timer khong keo nguoc ve gia tri cu.
        - Tat noi suy cua timer trong luc animate.
        - Gui lai lenh cuoi vai lan de controller chac chan nhan duoc lenh siet."""
        self.target_gripper_pos = target_pos
        if abs(self.current_gripper_pos - target_pos) <= 1e-4:
            self._send_gripper_joint_cmd(target_pos, -target_pos)
            return
        self._animating = True
        try:
            start = self.current_gripper_pos
            for index in range(1, steps + 1):
                position = start + (target_pos - start) * index / steps
                self.current_gripper_pos = position
                self._send_gripper_joint_cmd(position, -position)
                time.sleep(duration / steps)
            self.current_gripper_pos = target_pos
            for _ in range(5):
                self._send_gripper_joint_cmd(target_pos, -target_pos)
                time.sleep(0.05)
        finally:
            self._animating = False

    def open_gripper(self) -> str:
        """Mo rong ngon tay kep Industrial Gripper de san sang gap vat hoac nha vat tren khong."""
        self.node.get_logger().info("Thuc thi Skill: open_gripper() - Mo rong ngon tay kep")
        held = self.holding_object
        if held:
            if self.use_gazebo_attach:
                self._gazebo_detach(held)
            self._detach_object_from_robot(held)
        self._animate_gripper(self.grip_open_pos, steps=8, duration=0.22)
        if held:
            self.sync_collision_scene(exclude=held)
            self.holding_object = None
            self.grasp_offsets.pop(held, None)
        time.sleep(0.08)
        return "SUCCESS"

    def release_object_in_tray(self) -> str:
        """Mo nhe ngon tay kep du de tha khoi hop ma hoan toan khong cham vao thanh khay."""
        self.node.get_logger().info("Thuc thi: release_object_in_tray() - Nha vat nhe nhang trong khay")
        held = self.holding_object
        if held:
            if self.use_gazebo_attach:
                self._gazebo_detach(held)
            self._detach_object_from_robot(held)
        # Sau khi joint da nha, hai pad mo dan va cube tu nam xuong khay bang vat ly.
        self._animate_gripper(self.grip_open_pos, steps=8, duration=0.24)
        if held:
            self.sync_collision_scene(exclude=held)
            self.holding_object = None
            self.grasp_offsets.pop(held, None)
        time.sleep(0.12)
        return "SUCCESS"

    def close_gripper(self, object_name: str = None) -> str:
        """Co bop ngon tay kep Industrial Gripper ep chat vat the.

        Sau khi ngon kep da om khoi, tao DetachableJoint tren Gazebo de khoi thuc su
        bi nhac len cung tay may (khong chi 'nhac ao' tren RViz)."""
        self.node.get_logger().info(f"Thuc thi Skill: close_gripper(object={object_name}) - Co bop ngon kep")
        if object_name and not self._verify_grasp_alignment(object_name):
            self._animate_gripper(self.grip_open_pos, steps=6, duration=0.18)
            return "GRASP_FAILED"

        # Xoa tiep xuc cu; chi chap nhan message moi phat sinh trong lan dong nay.
        self._finger_contacts = {"left": set(), "right": set()}
        self._finger_contact_at = {"left": 0.0, "right": 0.0}
        self._finger_contact_positions = {"left": {}, "right": {}}

        if object_name:
            contact_ok = self._adaptive_gripper_close(object_name)
        else:
            self._animate_gripper(0.000, steps=10, duration=0.35)
            contact_ok = True

        if object_name:
            if not contact_ok:
                self.grasp_offsets.pop(object_name, None)
                self._animate_gripper(self.grip_open_pos, steps=8, duration=0.20)
                return "GRASP_FAILED"

            # DetachableJoint chi la lop on dinh cho simulator. Neu plugin nay
            # loi, van giu luc kep vat ly va tiep tuc; khong duoc tu dong mo hai
            # ngon lam roi vat vua kep duoc.
            gazebo_attached = False
            if self.use_gazebo_attach:
                gazebo_attached = self._gazebo_attach(object_name)
                if not gazebo_attached:
                    self.node.get_logger().warn(
                        f"Gazebo DetachableJoint khong phan hoi cho {object_name}; "
                        "tiep tuc bang luc ma sat cua hai pad"
                    )

            moveit_attached = False
            for attempt in range(3):
                if self._attach_object_to_robot(object_name):
                    moveit_attached = True
                    break
                time.sleep(0.12)
            if not moveit_attached:
                self.node.get_logger().error(f"MoveIt attach that bai: {object_name}")
                if gazebo_attached:
                    self._gazebo_detach(object_name)
                # Giu nguyen kep dong de vat khong roi; bao dung o lop scene.
                return "SCENE_FAILED"

            # Sau khi da xac nhan va attach, giu khe lenh 36 mm cho cube 40 mm.
            # Lenh nay duoc JointPositionController duy tri suot luc pick/place.
            self._animate_gripper(self.grip_hold_pos, steps=3, duration=0.09)
            self.holding_object = object_name
            self.node.get_logger().info(
                f"Da khoa grasp {object_name}; duy tri luc kep tai "
                f"q={self.grip_hold_pos:.4f} m"
            )
            for zone, occupant in self.zone_occupants.items():
                if occupant == object_name:
                    self.zone_occupants[zone] = None
        time.sleep(0.08)
        return "SUCCESS"

    def _tool_reached(self, target_pose: Pose, timeout_sec: float = 6.0) -> bool:
        """Xac nhan TCP do duoc (TF) that su toi dich truoc khi dong/nha kep.
        Tra ve False neu het thoi gian ma van chua toi."""
        deadline = time.monotonic() + timeout_sec
        last_error = None
        while time.monotonic() < deadline:
            try:
                transform = self.tf_buffer.lookup_transform(
                    "base_link", "tool0", rclpy.time.Time())
                actual = transform.transform.translation
                xy_error = math.hypot(actual.x - target_pose.position.x,
                                      actual.y - target_pose.position.y)
                z_error = abs(actual.z - target_pose.position.z - self.tcp_offset)
                last_error = (round(xy_error, 4), round(z_error, 4))
                if xy_error <= self.reach_tolerance and z_error <= self.reach_tolerance:
                    return True
            except Exception:
                pass
            time.sleep(0.10)
        self.node.get_logger().warn(f"TCP chua toi dich, sai so XY/Z = {last_error}")
        return False

    def _top_down_pose(self, x: float, y: float, z: float) -> Pose:
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation = Quaternion(
            x=self.operation_quaternion.x,
            y=self.operation_quaternion.y,
            z=self.operation_quaternion.z,
            w=self.operation_quaternion.w,
        )
        return pose

    def _lock_vertical_operation_orientation(self) -> bool:
        """Giu yaw hien tai nhung ep truc Z cua tool huong thang xuong ban.

        Quaternion do tu TF co the lech roll/pitch vai do do controller va IK.
        Neu tiep tuc tai su dung quaternion do, sai so se duoc mang xuong diem
        grasp va mot dau ngon se cham mat ban truoc. Ham nay tao lai quaternion
        chinh xac Rz(yaw) * Rx(pi), nen roll/pitch khong tich luy qua cac skill.
        """
        try:
            rotation = self.tf_buffer.lookup_transform(
                "base_link", "tool0", rclpy.time.Time()
            ).transform.rotation
            yaw = math.atan2(
                2.0 * (rotation.x * rotation.y + rotation.z * rotation.w),
                1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
            )
            half_yaw = 0.5 * yaw
            self.operation_quaternion = Quaternion(
                x=math.cos(half_yaw),
                y=math.sin(half_yaw),
                z=0.0,
                w=0.0,
            )
            return True
        except Exception as exc:
            self.node.get_logger().warn(
                f"Khong khoa duoc orientation thao tac tu TF: {exc}"
            )
            return False

    def _set_vertical_operation_yaw(self, yaw: float):
        """Dat orientation top-down voi yaw cho truoc, khong thay doi roll/pitch."""
        half_yaw = 0.5 * float(yaw)
        self.operation_quaternion = Quaternion(
            x=math.cos(half_yaw),
            y=math.sin(half_yaw),
            z=0.0,
            w=0.0,
        )

    def _operation_yaw(self) -> float:
        return 2.0 * math.atan2(
            self.operation_quaternion.y, self.operation_quaternion.x
        )

    @staticmethod
    def _nearest_symmetric_yaw(candidate: float, reference: float) -> float:
        """Chon yaw tuong duong theo chu ky pi gan yaw hien tai nhat.

        Hai ngon song song va cube vuong nen yaw va yaw+pi la cung mot grasp.
        """
        delta = (candidate - reference + 0.5 * math.pi) % math.pi - 0.5 * math.pi
        return reference + delta

    def _vertical_move_with_yaw_recovery(
            self, x: float, y: float, z: float,
            vel_scale: float, acc_scale: float,
            avoid_collisions: bool = None):
        """MoveL thang dung; neu IK ngoay co tay, doi yaw tren cao va tinh lai.

        Khong bao gio thuc thi trajectory da bi danh gia quay vong. Moi thay doi
        yaw dien ra tai travel height truoc khi ha xuong vat/khay.
        """
        target = self._top_down_pose(x, y, z)
        if self._move_cartesian(
                [target], avoid_collisions=avoid_collisions,
                vel_scale=vel_scale, acc_scale=acc_scale):
            return target

        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link", "tool0", rclpy.time.Time()
            ).transform
            clearance_z = float(transform.translation.z - self.tcp_offset)
        except Exception:
            return None

        original_yaw = self._operation_yaw()
        radial_yaw = math.atan2(float(y), float(x))
        raw_candidates = [radial_yaw, radial_yaw + 0.5 * math.pi]
        candidates = []
        for raw_yaw in raw_candidates:
            yaw = self._nearest_symmetric_yaw(raw_yaw, original_yaw)
            if abs(yaw - original_yaw) >= math.radians(5.0) and all(
                    abs(yaw - previous) >= math.radians(5.0)
                    for previous in candidates):
                candidates.append(yaw)

        for yaw in candidates:
            self._set_vertical_operation_yaw(yaw)
            rotate_pose = self._top_down_pose(x, y, clearance_z)
            self.node.get_logger().warn(
                f"IK duong thang khong tot; doi yaw an toan "
                f"{math.degrees(yaw):.1f} do tai z={clearance_z:.3f} m"
            )
            if not self._move_cartesian(
                    [rotate_pose], avoid_collisions=avoid_collisions,
                    vel_scale=min(0.25, self.carry_vel_scale),
                    acc_scale=min(0.18, self.carry_acc_scale)):
                continue

            target = self._top_down_pose(x, y, z)
            if self._move_cartesian(
                    [target], avoid_collisions=avoid_collisions,
                    vel_scale=vel_scale, acc_scale=acc_scale):
                return target

        # Giu orientation cuoi cung robot da thuc su dat. Neu khong candidate nao
        # chay duoc thi khoi phuc target logic ban dau cho skill tiep theo.
        self._lock_vertical_operation_orientation()
        return None

    def _travel_to(self, x: float, y: float) -> bool:
        """Nang thang va di ngang o do cao an toan travel_height, giu nguyen nhanh IK.

        Khi robot moi khoi dong, dua ve joint posture home da biet truoc bang MoveJ.
        Tu posture do tro di, cac doan di tren ban dung MoveL Cartesian ngan, de tranh
        OMPL chon mot nhanh IK gap gripper nguoc vao forearm."""
        clearance = self.travel_height
        transform = None
        orientation_ready = False
        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link", "tool0", rclpy.time.Time()
            ).transform
            rotation = transform.rotation
            tool_z_world_z = 1.0 - 2.0 * (
                rotation.x * rotation.x + rotation.y * rotation.y
            )
            # Cho phep toi da 2 do. Lon hon muc nay co the lam mot dau ngon
            # thap hon dau con lai va va vao mat khay khi ha xuong.
            orientation_ready = tool_z_world_z <= -math.cos(math.radians(2.0))
        except Exception:
            pass

        if orientation_ready:
            self._lock_vertical_operation_orientation()
            target = self._top_down_pose(x, y, clearance)
            waypoints = []
            tool = transform.translation
            current_tcp_z = tool.z - self.tcp_offset
            if current_tcp_z < clearance - 0.01:
                waypoints.append(self._top_down_pose(tool.x, tool.y, clearance))
            waypoints.append(target)
            # Moi waypoint dung orientation thang dung da chuan hoa; khong sao
            # chep quaternion TF co sai so nghieng vao duong MoveL.
            if self._move_cartesian(waypoints, preserve_orientation=False):
                return True

        # Pose khoi dong mac dinh cua Gazebo khong phai posture thao tac. Neu dua
        # thang pose dich cho OMPL, nghiem IK co the gap gripper vao forearm. MoveJ
        # ve home truoc tao mot nghiem seed an toan va chi can thuc hien mot lan.
        if not orientation_ready:
            self.node.get_logger().info(
                "TCP chua o posture thao tac; MoveJ ve home truoc khi MoveL"
            )
            if self._move_to_joint_target(self.home_joints):
                time.sleep(0.15)  # cho TF nhan joint state cuoi cua controller
                self._lock_vertical_operation_orientation()
                home_waypoints = []
                try:
                    transform = self.tf_buffer.lookup_transform(
                        "base_link", "tool0", rclpy.time.Time()
                    ).transform
                    tool = transform.translation
                    current_tcp_z = tool.z - self.tcp_offset
                    if current_tcp_z < clearance - 0.01:
                        home_waypoints.append(
                            self._top_down_pose(tool.x, tool.y, clearance)
                        )
                except Exception:
                    pass
                target = self._top_down_pose(x, y, clearance)
                home_waypoints.append(target)
                if self._move_cartesian(
                        home_waypoints, preserve_orientation=False):
                    return True
                self.node.get_logger().warn(
                    "MoveJ home thanh cong nhung MoveL toi diem thao tac that bai"
                )
            else:
                self.node.get_logger().warn(
                    "Khong dua duoc robot ve posture home an toan"
                )
        else:
            self.node.get_logger().warn(
                "MoveL transfer khong thanh cong; dung MoveGroup fallback co kiem tra goc quay"
            )
        target = self._top_down_pose(x, y, clearance)
        return self._move_to_pose_target(target)

    def move_above(self, target_name: str) -> str:
        pos = self._get_target_xy(target_name)
        if pos is None:
            return "INVALID_OBJECT"
        with self._busy():
            return "SUCCESS" if self._travel_to(pos[0], pos[1]) else "PLANNING_FAILED"

    def move_to_zone(self, zone_name: str) -> str:
        """Di chuyen tay kep den phia tren vung Zone."""
        self.node.get_logger().info(f"Thuc thi Skill: move_to_zone({zone_name})")
        return self.move_above(zone_name)

    def pick(self, object_name: str) -> str:
        """
        Chu trinh gap vat:
        1. Kiem tra hop le (+ camera neu require_camera)
        2. Mo kep, loai vat khoi collision scene
        3. Tiep can tren khong o travel_height
        4. Ha Cartesian thang dung (KHONG fallback OMPL de tranh lat khop)
        5. Chi dong kep khi TF xac nhan da toi dung vi tri
        6. Nhac thang dung len travel_height (cham hon khi dang cam vat)
        """
        self.node.get_logger().info(f"Thuc thi Skill: pick({object_name})")
        if object_name not in self.objects_config:
            self.node.get_logger().error(f"pick: Vat the '{object_name}' khong ton tai!")
            return "INVALID_OBJECT"

        if self.holding_object is not None:
            self.node.get_logger().error(f"pick: Robot dang giu '{self.holding_object}', khong the gap them!")
            return "FAILED"

        if self.require_camera and not self.get_scene_state()["camera_ready"]:
            self.node.get_logger().error("pick: camera chua san sang, tu choi gap vat")
            return "CAMERA_NOT_READY"

        # Chup snapshot toa do MOT LAN, dung xuyen suot chu trinh pick
        snapshot = self.object_positions.get(object_name)
        if snapshot is None:
            return "INVALID_OBJECT"
        obj_x, obj_y, obj_z = map(float, snapshot[:3])

        with self._busy():
            # Giai phong khoi zone cu neu vat tung duoc dat o zone do
            for z, occupant in list(self.zone_occupants.items()):
                if occupant == object_name:
                    self.zone_occupants[z] = None

            # 1. Mo kep va loai target khoi collision scene de co the cham vao no.
            self.open_gripper()
            if not self.sync_collision_scene(exclude=object_name):
                return "SCENE_FAILED"

            # 2. Tiep can tren khong cua vat o do cao an toan travel_height
            if not self._travel_to(obj_x, obj_y):
                return "PLANNING_FAILED"

            # 3. Ha kep Cartesian thang dung xuong vat (khong fallback)
            # Pad om cube o cao do an toan, dau ngon khong ha xuong mat khay.
            grasp_pose = self._vertical_move_with_yaw_recovery(
                obj_x, obj_y, obj_z + self.grasp_center_offset,
                vel_scale=self.approach_vel_scale,
                acc_scale=self.approach_acc_scale,
            )
            if grasp_pose is None:
                self.node.get_logger().error("pick: khong ha thang duoc xuong vat")
                return "PLANNING_FAILED"
            if not self._tool_reached(grasp_pose):
                return "GRASP_NOT_REACHED"

            time.sleep(0.25)

            # 4. Dong kep om chat vat (+ gan khop co dinh tren Gazebo)
            close_status = self.close_gripper(object_name)
            if close_status != "SUCCESS":
                return close_status
            time.sleep(0.15)
            self._publish_dynamic_cube_state()

            # 5. Nhac vat thang dung len do cao chuyen ngang (cham vi dang cam vat)
            lift_pose = self._top_down_pose(obj_x, obj_y, self.travel_height)
            if not self._move_cartesian(
                    [lift_pose], vel_scale=self.lift_vel_scale,
                    acc_scale=self.lift_acc_scale):
                return "RETRACT_FAILED"

            dx, dy, dz = self.grasp_offsets.get(
                object_name, (0.0, 0.0, -self.grasp_center_offset)
            )
            self.object_positions[object_name] = [
                obj_x + dx, obj_y + dy, self.travel_height + dz
            ]
            self._publish_dynamic_cube_state()
        return "SUCCESS"

    def place(self, object_name: str, zone_name: str) -> str:
        """Dat vat dang kep vao Zone (xem _place_at_xy)."""
        self.node.get_logger().info(f"Thuc thi Skill: place({object_name}, {zone_name})")
        if object_name not in self.objects_config:
            return "INVALID_OBJECT"
        if zone_name not in self.zones_config:
            return "INVALID_OBJECT"
        if self.holding_object != object_name:
            self.node.get_logger().error(
                f"place: Robot khong giu vat '{object_name}' (hien tai: '{self.holding_object}')!")
            return "FAILED"

        zone_pos = self.zones_config[zone_name].get("position", [0.35, 0.0, 0.001])
        occupied_by = self.zone_occupants.get(zone_name)
        if occupied_by not in (None, object_name):
            return "ZONE_OCCUPIED"

        with self._busy():
            return self._place_at_xy(object_name, zone_pos[0], zone_pos[1], zone_name)

    def _place_at_xy(self, object_name: str, target_x: float, target_y: float, zone_name: str = None) -> str:
        """Di chuyen ngang -> ha thang -> kiem tra toi dich -> nha -> nhac len."""
        # 1. Di chuyen ngang tren khong
        if not self._travel_to(target_x, target_y):
            return "PLANNING_FAILED"

        dx, dy, dz = self.grasp_offsets.get(
            object_name, (0.0, 0.0, -self.grasp_center_offset)
        )
        self.object_positions[object_name] = [
            target_x + dx, target_y + dy, self.travel_height + dz
        ]
        self._publish_dynamic_cube_state()

        # 2. Ha vat xuong mat ban (Cartesian thang dung, khong fallback)
        place_pose = self._vertical_move_with_yaw_recovery(
            target_x, target_y, self.place_height,
            vel_scale=self.approach_vel_scale,
            acc_scale=self.approach_acc_scale,
        )
        if place_pose is None:
            return "PLANNING_FAILED"
        if not self._tool_reached(place_pose):
            return "PLACE_NOT_REACHED"

        time.sleep(0.35)

        # 3. Nha vat nhe nhang
        release_status = self.release_object_in_tray()
        if release_status != "SUCCESS":
            return release_status

        self.object_positions[object_name] = [
            target_x, target_y, self.cube_rest_height
        ]
        if zone_name is not None:
            self.zone_occupants[zone_name] = object_name
        self._publish_dynamic_cube_state()

        # 4. Nhac kep thang dung len
        lift_pose = self._top_down_pose(target_x, target_y, self.travel_height)
        if not self._move_cartesian(
                [lift_pose], vel_scale=self.lift_vel_scale,
                acc_scale=self.lift_acc_scale):
            return "RETRACT_FAILED"

        # 5. Gripper da mo khi release; chi cap nhat collision scene, khong tao
        # them mot chu ky dong/mo khong can thiet.
        if not self.sync_collision_scene():
            return "SCENE_FAILED"
        return "SUCCESS"

    # =========================================================================
    # --- CAC SKILL NANG CAO (ADVANCED SKILLS) ---
    # =========================================================================

    def swap(self, object_a: str, object_b: str) -> str:
        """
        Hoan doi vi tri 2 vat the thong qua mot vung dem con trong.
        Vat o khay nguon (khong thuoc zone nao) se duoc tra ve DUNG toa do ban dau cua no.
        """
        self.node.get_logger().info(f"Thuc thi Skill nang cao: swap({object_a}, {object_b})")
        if object_a not in self.objects_config or object_b not in self.objects_config:
            return "INVALID_OBJECT"
        if object_a == object_b:
            return "INVALID_OBJECT"

        zone_of_a = None
        zone_of_b = None
        for z, occupant in self.zone_occupants.items():
            if occupant == object_a:
                zone_of_a = z
            if occupant == object_b:
                zone_of_b = z

        pos_a = list(self.object_positions[object_a])
        pos_b = list(self.object_positions[object_b])

        buffer_zone = self.find_free_position()
        if buffer_zone is None:
            return "NO_FREE_POSITION"

        def _put(obj, zone, xy):
            if zone is not None:
                return self.place(obj, zone)
            with self._busy():
                return self._place_at_xy(obj, xy[0], xy[1], None)

        # 1. Chuyen A vao vung dem
        if self.pick(object_a) != "SUCCESS":
            return "FAILED"
        if self.place(object_a, buffer_zone) != "SUCCESS":
            return "FAILED"

        # 2. Chuyen B vao vi tri cu cua A
        if self.pick(object_b) != "SUCCESS":
            return "FAILED"
        if _put(object_b, zone_of_a, pos_a) != "SUCCESS":
            return "FAILED"

        # 3. Chuyen A tu vung dem vao vi tri cu cua B
        if self.pick(object_a) != "SUCCESS":
            return "FAILED"
        if _put(object_a, zone_of_b, pos_b) != "SUCCESS":
            return "FAILED"

        self.home()
        return "SUCCESS"

    def stack(self, object_top: str, object_bottom: str) -> str:
        """Xep chong object_top len tren dinh object_bottom."""
        self.node.get_logger().info(f"Thuc thi Skill nang cao: stack({object_top}, {object_bottom})")
        if object_top not in self.objects_config or object_bottom not in self.objects_config:
            return "INVALID_OBJECT"
        if object_top == object_bottom:
            return "INVALID_OBJECT"

        pos_bottom = self.object_positions.get(object_bottom)
        if not pos_bottom:
            return "INVALID_OBJECT"
        bx, by = float(pos_bottom[0]), float(pos_bottom[1])
        bottom_height = float(self.objects_config[object_bottom]["size"][2])
        stack_z = self.place_height + bottom_height

        # 1. Gap khoi tren
        if self.pick(object_top) != "SUCCESS":
            return "FAILED"

        with self._busy():
            # 2. Di chuyen tren khong toi ngay tren dinh khoi duoi
            if not self._travel_to(bx, by):
                return "PLANNING_FAILED"

            # 3. Ha xuong do cao xep chong (khong tranh va cham vi se tiep xuc voi khoi duoi)
            stack_pose = self._vertical_move_with_yaw_recovery(
                bx, by, stack_z,
                avoid_collisions=False,
                vel_scale=self.approach_vel_scale,
                acc_scale=self.approach_acc_scale,
            )
            if stack_pose is None:
                return "PLANNING_FAILED"
            if not self._tool_reached(stack_pose):
                return "PLACE_NOT_REACHED"
            time.sleep(0.35)

            # 4. Nha kep
            release_status = self.release_object_in_tray()
            if release_status != "SUCCESS":
                return release_status
            self.object_positions[object_top] = [
                bx, by, self.cube_rest_height + bottom_height
            ]
            self._publish_dynamic_cube_state()

            # 5. Nhac kep thang dung len cao
            lift_pose = self._top_down_pose(bx, by, self.travel_height + bottom_height)
            if not self._move_cartesian(
                    [lift_pose], avoid_collisions=False,
                    vel_scale=self.lift_vel_scale,
                    acc_scale=self.lift_acc_scale):
                return "RETRACT_FAILED"
            self.open_gripper()
            if not self.sync_collision_scene():
                return "SCENE_FAILED"
        return "SUCCESS"

    def _return_object_to_tray(self, object_name: str) -> str:
        """Gap 1 vat the ra khoi vung va hoan tra ve khay chua phoi ban dau."""
        if object_name not in self.objects_config:
            return "INVALID_OBJECT"

        init_pos = self.objects_config[object_name].get("initial_position", [0.24, 0.0, 0.02])

        pick_status = self.pick(object_name)
        if pick_status != "SUCCESS":
            return pick_status

        with self._busy():
            status = self._place_at_xy(object_name, init_pos[0], init_pos[1], None)
            if status != "SUCCESS":
                return status
            self.object_positions[object_name] = list(init_pos)
            self._publish_dynamic_cube_state()
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
        """Tra ve vi tri logic cua tung vat the ('zone_a', 'zone_b', ... hoac 'source_tray')."""
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
            return list(self.object_positions[name][:2])
        if name in self.zones_config:
            return list(self.zones_config[name].get("position")[:2])
        return None

    def _wait_for_future(self, future, timeout_sec: float = 10.0) -> bool:
        """Cho future hoan thanh ma khong gay deadlock hoac xung dot spin."""
        start_time = time.time()
        while not future.done() and (time.time() - start_time < timeout_sec):
            time.sleep(0.015)
        return future.done()

    def _fill_start_state(self, start_state):
        """Dat start state = khop do duoc hien tai, nhung giu is_diff=True de
        khong mat vat dang duoc attach vao tool0."""
        start_state.is_diff = True
        if all(name in self.current_joint_positions for name in self.joint_names):
            start_state.joint_state.name = list(self.joint_names)
            start_state.joint_state.position = [
                float(self.current_joint_positions[name]) for name in self.joint_names
            ]

    def _trajectory_max_joint_travel(self, robot_traj) -> float:
        """Tong goc quay that lon nhat cua mot khop doc theo quy dao.

        Joint revolute co the duoc bieu dien tu +pi sang -pi. Dung shortest
        angular distance de khong tinh nham bien bieu dien thanh mot vong quay,
        nhung van cong don de bat duong IK that su ngoay nhieu vong.
        """
        points = robot_traj.joint_trajectory.points
        if len(points) < 2:
            return 0.0
        n = len(points[0].positions)
        travel = [0.0] * n
        for p1, p2 in zip(points, points[1:]):
            for j in range(n):
                raw_delta = float(p2.positions[j] - p1.positions[j])
                shortest_delta = (raw_delta + math.pi) % (2.0 * math.pi) - math.pi
                travel[j] += abs(shortest_delta)
        return max(travel)

    def _ensure_timing(self, traj, max_vel: float = 0.5):
        """Chi retime khi MoveIt khong tra ve timestamp hop le; neu co thi giu nguyen."""
        points = traj.points
        if len(points) < 2:
            return
        valid = True
        last_t = -1.0
        for p in points:
            t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
            if t <= last_t:
                valid = False
                break
            last_t = t
            if len(p.velocities) != len(p.positions):
                valid = False
                break
        if valid and last_t > 0.0:
            return

        self.node.get_logger().warn("Trajectory khong co timestamp hop le - retime thu cong")
        times = [0.0]
        for i in range(1, len(points)):
            dt = 0.025
            for a, b in zip(points[i - 1].positions, points[i].positions):
                dt = max(dt, abs(b - a) / max_vel)
            times.append(times[-1] + dt)
        for i, p in enumerate(points):
            p.time_from_start.sec = int(times[i])
            p.time_from_start.nanosec = int((times[i] % 1.0) * 1e9)
            if i == 0 or i == len(points) - 1:
                p.velocities = [0.0] * len(p.positions)
            else:
                span = times[i + 1] - times[i - 1]
                p.velocities = [
                    (points[i + 1].positions[k] - points[i - 1].positions[k]) / span
                    for k in range(len(p.positions))
                ]
            p.accelerations = [0.0] * len(p.positions)

    def _execute_robot_trajectory(self, robot_traj, final_pose: Pose = None) -> bool:
        """Thuc thi RobotTrajectory bang ExecuteTrajectory."""
        if not self._execute_traj_client.wait_for_server(timeout_sec=3.0):
            self.node.get_logger().error("ExecuteTrajectory action server khong phan hoi")
            return False
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = robot_traj
        send_future = self._execute_traj_client.send_goal_async(goal)
        if not self._wait_for_future(send_future, timeout_sec=10.0):
            self.node.get_logger().error("Het thoi gian gui quy dao den MoveIt")
            return False
        handle = send_future.result()
        if not handle or not handle.accepted:
            self.node.get_logger().error("MoveIt tu choi thuc thi quy dao")
            return False
        result_future = handle.get_result_async()
        if not self._wait_for_future(result_future, timeout_sec=90.0):
            self.node.get_logger().error("Het thoi gian cho controller thuc thi quy dao")
            return False
        result = result_future.result()
        if not result:
            self.node.get_logger().error("Controller khong tra ve ket qua thuc thi")
            return False
        err_val = result.result.error_code.val
        if err_val == 1:
            return True
        if err_val == -4 and final_pose is not None and self._tool_reached(final_pose, 2.0):
            self.node.get_logger().warn("Controller bao sai so khop, nhung TF xac nhan TCP da toi dich")
            return True
        self.node.get_logger().error(
            f"Thuc thi quy dao that bai, MoveIt error_code={err_val}"
        )
        return False

    def _move_to_joint_target(self, joint_values: list) -> bool:
        """Lap ke hoach va thuc thi dich khop toi cac goc hop le da cau hinh."""
        if not self._move_group_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("MoveGroup action server khong phan hoi!")
            return False

        targets = self._unwrap_joint_target(joint_values)

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        # De trong pipeline/planner id de MoveIt chon pipeline mac dinh.
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 8.0
        goal_msg.request.max_velocity_scaling_factor = 0.65
        goal_msg.request.max_acceleration_scaling_factor = 0.45
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        self._fill_start_state(goal_msg.request.start_state)

        constraints = Constraints()
        for name, val in zip(self.joint_names, targets):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(val)
            jc.tolerance_above = 0.02
            jc.tolerance_below = 0.02
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        goal_msg.request.goal_constraints.append(constraints)

        send_goal_future = self._move_group_client.send_goal_async(goal_msg)
        if not self._wait_for_future(send_goal_future, timeout_sec=10.0):
            self.node.get_logger().error("Het thoi gian gui joint goal den MoveIt")
            return False

        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.node.get_logger().warn("Joint goal bi tu choi hoac timeout.")
            return False

        get_result_future = goal_handle.get_result_async()
        if not self._wait_for_future(get_result_future, timeout_sec=120.0):
            self.node.get_logger().error("Het thoi gian cho ket qua joint goal")
            return False

        result = get_result_future.result()
        return bool(result and result.result.error_code.val == 1)

    def _move_to_pose_target(self, target_pose: Pose) -> bool:
        """Plan (plan_only) toi pose, KIEM TRA quy dao khong quay vong, roi moi thuc thi.

        - Huong kep bi khoa chat (cung huong voi duong Cartesian) de buoc Cartesian ke tiep
          khong phai xoay co tay.
        - Chi dat pose dich. Khong ghep them joint constraints vao cung goal vi dieu do
          ep IK phai thoa dong thoi hai dich thuong mau thuan va khong tim duoc goal.
        - Quy dao co khop quay > max_joint_travel se bi tu choi.
        - Khi dang cam vat: giam toc do / gia toc."""
        if not self._move_group_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("MoveGroup action server khong phan hoi!")
            return False

        if self.holding_object:
            vel_scale, acc_scale = self.carry_vel_scale, self.carry_acc_scale
        else:
            vel_scale, acc_scale = 0.55, 0.35

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        goal_msg.request.num_planning_attempts = 12
        goal_msg.request.allowed_planning_time = 8.0
        goal_msg.request.max_velocity_scaling_factor = vel_scale
        goal_msg.request.max_acceleration_scaling_factor = acc_scale
        goal_msg.planning_options.plan_only = True
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        self._fill_start_state(goal_msg.request.start_state)

        # Pose cua tool0 (cong them offset kep)
        goal_pose = Pose()
        goal_pose.position.x = target_pose.position.x
        goal_pose.position.y = target_pose.position.y
        goal_pose.position.z = target_pose.position.z + self.tcp_offset
        goal_pose.orientation = target_pose.orientation

        # Rang buoc vi tri: cau ban kinh 8mm (truoc day 25mm gay lech khi gap)
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "base_link"
        pos_constraint.link_name = "tool0"
        bv = BoundingVolume()
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.008]
        bv.primitives.append(sp)
        bv.primitive_poses.append(goal_pose)
        pos_constraint.constraint_region = bv
        pos_constraint.weight = 1.0

        # Rang buoc huong: giu kep thang dung, yaw gan nhu khoa
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = "base_link"
        orient_constraint.link_name = "tool0"
        orient_constraint.orientation = target_pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.05
        orient_constraint.absolute_y_axis_tolerance = 0.05
        orient_constraint.absolute_z_axis_tolerance = 0.10
        orient_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(orient_constraint)

        goal_msg.request.goal_constraints.append(constraints)

        result = None
        # OMPL co tinh ngau nhien. Mot duong co the bi bo sau buoc time
        # parameterization neu cham sat forearm; yeu cau lai se tao cay tim kiem moi.
        for attempt in range(1, 4):
            send_goal_future = self._move_group_client.send_goal_async(goal_msg)
            if not self._wait_for_future(send_goal_future, timeout_sec=10.0):
                self.node.get_logger().error("Het thoi gian gui pose goal den MoveIt")
                return False
            goal_handle = send_goal_future.result()
            if not goal_handle or not goal_handle.accepted:
                self.node.get_logger().error("MoveIt tu choi pose goal")
                return False

            get_result_future = goal_handle.get_result_async()
            if not self._wait_for_future(get_result_future, timeout_sec=30.0):
                self.node.get_logger().error("Het thoi gian cho MoveIt lap ke hoach pose")
                return False
            result = get_result_future.result()
            if result and result.result.error_code.val == 1:
                break

            error_code = result.result.error_code.val if result else "NO_RESULT"
            if attempt < 3:
                self.node.get_logger().warn(
                    f"Pose plan lan {attempt}/3 loi {error_code}; lap lai cay OMPL"
                )
                time.sleep(0.05)

        if not result or result.result.error_code.val != 1:
            error_code = result.result.error_code.val if result else "NO_RESULT"
            self.node.get_logger().error(
                f"Khong lap duoc ke hoach pose sau 3 lan, MoveIt error_code={error_code}"
            )
            return False

        planned = result.result.planned_trajectory
        if not planned.joint_trajectory.points:
            return False

        travel = self._trajectory_max_joint_travel(planned)
        if travel > self.max_joint_travel:
            self.node.get_logger().error(
                f"Tu choi quy dao: 1 khop quay tong cong {travel:.2f} rad > {self.max_joint_travel:.2f} rad")
            return False

        self._ensure_timing(planned.joint_trajectory)
        return self._execute_robot_trajectory(planned, final_pose=target_pose)

    def _move_cartesian(self, waypoints: list, min_fraction: float = None,
                        avoid_collisions: bool = None, vel_scale: float = None,
                        acc_scale: float = None,
                        preserve_orientation: bool = False) -> bool:
        """Di chuyen duong thang Cartesian. Chi chap nhan khi hoan thanh >= min_fraction,
        khong co buoc nhay khop, va khong quay vong.

        Neu khong truyen vel_scale/acc_scale: dung toc do cham (carry_*) khi dang cam vat,
        nguoc lai dung toc do thuong (0.35 / 0.20) de khoi khong bi truot do gia toc."""
        if min_fraction is None:
            min_fraction = self.cartesian_fraction
        if avoid_collisions is None:
            avoid_collisions = self.cartesian_avoid_collisions
        if vel_scale is None:
            vel_scale = self.carry_vel_scale if self.holding_object else self.cartesian_max_vel
        if acc_scale is None:
            acc_scale = self.carry_acc_scale if self.holding_object else self.cartesian_max_acc

        if not self._cartesian_path_client.wait_for_service(timeout_sec=2.0):
            return False

        req = GetCartesianPath.Request()
        req.header.frame_id = "base_link"
        req.header.stamp = self.node.get_clock().now().to_msg()
        req.group_name = "ur_manipulator"
        req.link_name = "tool0"
        self._fill_start_state(req.start_state)

        # Mac dinh dung quaternion thao tac thang dung trong waypoint. Tuy chon
        # preserve_orientation chi danh cho chuyen dong khong phai pick/place.
        measured_orientation = None
        if preserve_orientation:
            try:
                rotation = self.tf_buffer.lookup_transform(
                    "base_link", "tool0", rclpy.time.Time()
                ).transform.rotation
                measured_orientation = Quaternion(
                    x=rotation.x, y=rotation.y, z=rotation.z, w=rotation.w
                )
            except Exception:
                measured_orientation = None

        # Offset z cho tool0 do co gripper
        offset_wps = []
        for wp in waypoints:
            p = Pose()
            p.position.x = wp.position.x
            p.position.y = wp.position.y
            p.position.z = wp.position.z + self.tcp_offset
            p.orientation = measured_orientation or wp.orientation
            offset_wps.append(p)

        req.waypoints = offset_wps
        # 2 mm cho doan tiep can; 4 mm cho doan di tren khong de giam so diem
        # trajectory ma van min, khong anh huong do chinh xac diem cuoi.
        req.max_step = 0.002 if vel_scale <= self.approach_vel_scale + 0.02 else 0.004
        req.jump_threshold = 0.0
        req.prismatic_jump_threshold = 0.0
        req.revolute_jump_threshold = 0.25
        req.avoid_collisions = bool(avoid_collisions)
        req.max_velocity_scaling_factor = float(vel_scale)
        req.max_acceleration_scaling_factor = float(acc_scale)

        future = self._cartesian_path_client.call_async(req)
        if not self._wait_for_future(future, timeout_sec=5.0):
            self.node.get_logger().error("Het thoi gian tinh Cartesian path")
            return False

        res = future.result()
        if not res or not res.solution.joint_trajectory.points:
            self.node.get_logger().warn("MoveIt khong tra ve Cartesian path hop le")
            return False
        if res.fraction < min_fraction:
            self.node.get_logger().warn(
                f"Cartesian path chi dat {res.fraction * 100.0:.1f}%, can it nhat {min_fraction * 100.0:.0f}%"
            )
            return False

        traj = res.solution.joint_trajectory

        # Chan buoc nhay khop dot ngot (singularity / lat nhanh)
        for p1, p2 in zip(traj.points, traj.points[1:]):
            if any(
                abs((float(b - a) + math.pi) % (2.0 * math.pi) - math.pi) > 0.30
                for a, b in zip(p1.positions, p2.positions)
            ):
                self.node.get_logger().warn("Cartesian path co buoc nhay khop bat thuong >0.30 rad")
                return False

        # Chan quay vong: tong goc quay cua 1 khop qua lon
        travel = self._trajectory_max_joint_travel(res.solution)
        if travel > self.max_joint_travel:
            self.node.get_logger().warn(
                f"Cartesian path quay khop tong cong {travel:.2f} rad > {self.max_joint_travel:.2f} rad")
            return False

        self._ensure_timing(traj)
        return self._execute_robot_trajectory(res.solution, final_pose=waypoints[-1] if waypoints else None)

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
        # Trong frame tool0, +Z huong xuong khi grasp top-down. Cube nam thap
        # hon TCP dung bang sai lech tam da xac nhan truoc luc kep.
        dz = self.grasp_offsets.get(
            object_name, (0.0, 0.0, -self.grasp_center_offset)
        )[2]
        pose.position.z = self.tcp_offset - float(dz)
        pose.orientation.w = 1.0
        attached.object.primitive_poses.append(pose)
        attached.touch_links = [
            "tool0", "gripper_flange_link", "gripper_base_link",
            "gripper_left_finger", "gripper_right_finger",
            "flange", "ft_frame"
        ]
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
