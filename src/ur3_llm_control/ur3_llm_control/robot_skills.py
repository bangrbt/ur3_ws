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
- reset_scene(): Dua tat ca cac khoi hop ve lai 3 khay phoi ban dau
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
from std_msgs.msg import String
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

        # Action Clients MoveIt 2
        self._move_group_client = ActionClient(self.node, MoveGroup, "/move_action")
        self._execute_traj_client = ActionClient(self.node, ExecuteTrajectory, "/execute_trajectory")
        self._cartesian_path_client = self.node.create_client(GetCartesianPath, "/compute_cartesian_path")
        self._planning_scene_client = self.node.create_client(ApplyPlanningScene, "/apply_planning_scene")

        # Dynamic Cube State Publisher
        self.cube_state_pub = self.node.create_publisher(String, "/scene/cube_states", 10)

        # Joint States Subscription & Tracking
        self.current_joint_positions = {}
        self.joint_sub = self.node.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 10
        )

        # TF Buffer & Listener for gripper tracking
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        # Timer dinh ky theo doi tool0 khi dang gap vat de cap nhat vi tri vat the
        self.tracking_timer = self.node.create_timer(0.1, self._tracking_callback)

        # Orientation chu vi thang dung vuong goc mat ban (Top-down grasp)
        # Khau tac dong cuoi quay huong xuong ban: Pitch = 180 do
        self.top_down_quaternion = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)

        # Tham so chieu cao
        self.approach_height = float(self.motion_params.get("approach_height", 0.14))
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
    # --- HELPER DONG BO GAZEBO VA TF ---
    # =========================================================================

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

    def _set_gazebo_model_pose(self, name: str, x: float, y: float, z: float):
        """Cap nhat vi tri vat the trong Gazebo (Ignition) bang service set_pose."""
        def _call():
            cmd = [
                "ign", "service",
                "-s", "/world/default/set_pose",
                "--reqtype", "ignition.msgs.Pose",
                "--reptype", "ignition.msgs.Boolean",
                "--timeout", "300",
                "--req", f'name: "{name}", position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, orientation: {{w: 1.0}}'
            ]
            try:
                subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=0.6)
            except Exception:
                pass
        threading.Thread(target=_call, daemon=True).start()

    def _unwrap_joint_target(self, goal_joints: list) -> list:
        """Chuyen doi goal joint ve goc quay gan nhat de khong bi quay xoay 360 do."""
        unwrapped = []
        for name, goal in zip(self.joint_names, goal_joints):
            current = self.current_joint_positions.get(name, goal)
            diff = (goal - current + math.pi) % (2.0 * math.pi) - math.pi
            target = current + diff
            target = max(-6.20, min(6.20, target))
            unwrapped.append(target)
        return unwrapped

    # =========================================================================
    # --- CAC SKILL NGUYEN THUY ---
    # =========================================================================

    def home(self) -> str:
        """Dua tay may ve tu the cho mac dinh (Home pose) voi goc ngan nhat khong xoay vong."""
        self.node.get_logger().info("Thuc thi Skill: home()")
        success = self._move_to_joint_target(self.home_joints)
        return "SUCCESS" if success else "PLANNING_FAILED"

    def open_gripper(self) -> str:
        """Mo kep va nha vat the khoi PlanningScene (neu dang giu)."""
        self.node.get_logger().info("Thuc thi Skill: open_gripper()")
        if self.holding_object:
            self._detach_object_from_robot(self.holding_object)
            self.holding_object = None
        time.sleep(0.15)
        return "SUCCESS"

    def close_gripper(self, object_name: str = None) -> str:
        """Dong kep va dinh kem vat the vao PlanningScene robot."""
        self.node.get_logger().info(f"Thuc thi Skill: close_gripper(object={object_name})")
        if object_name:
            self._attach_object_to_robot(object_name)
            self.holding_object = object_name
        time.sleep(0.15)
        return "SUCCESS"

    def move_above(self, target_name: str) -> str:
        """Di chuyen tay kep den vi tri tren khong cach vat the/zone 14cm."""
        self.node.get_logger().info(f"Thuc thi Skill: move_above({target_name})")
        pos = self._get_target_xy(target_name)
        if pos is None:
            return "INVALID_OBJECT"

        target_pose = Pose()
        target_pose.position.x = pos[0]
        target_pose.position.y = pos[1]
        target_pose.position.z = self.approach_height
        target_pose.orientation = self.top_down_quaternion

        if not self._move_cartesian([target_pose]):
            if not self._move_to_pose_target(target_pose):
                return "PLANNING_FAILED"
        return "SUCCESS"

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
        7. Nhac vat len cao (Cartesian thang dung)
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

        # 1. Mo kep
        self.open_gripper()

        # 2. Tiep can tren khong cua vat
        approach_pose = Pose()
        approach_pose.position.x = obj_xy[0]
        approach_pose.position.y = obj_xy[1]
        approach_pose.position.z = self.approach_height
        approach_pose.orientation = self.top_down_quaternion

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

        # 4. Dong kep (Dinh kem vat the & dong bo vi tri Gazebo)
        self.close_gripper(object_name)
        self._set_gazebo_model_pose(object_name, obj_xy[0], obj_xy[1], self.grasp_height)
        self._publish_dynamic_cube_state()

        # 5. Nhac vat len cao (Cartesian thang dung)
        if not self._move_cartesian([approach_pose]):
            self._move_to_pose_target(approach_pose)

        self._set_gazebo_model_pose(object_name, obj_xy[0], obj_xy[1], self.approach_height)
        self.object_positions[object_name] = [obj_xy[0], obj_xy[1], self.approach_height]
        self._publish_dynamic_cube_state()

        return "SUCCESS"

    def place(self, object_name: str, zone_name: str) -> str:
        """
        Chu trinh dat vat hoan chinh:
        1. Kiem tra tinh hop le
        2. Tinh toan toa do dat (tu dong tranh chong de neu o da co vat khac)
        3. Di chuyen tren khong toi Zone (Cartesian Transfer ngang)
        4. Ha vat xuong mat ban (Cartesian Descend)
        5. Mo kep (Nha vat & dat on dinh trong Gazebo)
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

        # Neu Zone da co vat khac, dat lech 0.032m theo truc Y de khong bi chong de len nhau
        occupied_by = self.zone_occupants.get(zone_name)
        offset_y = 0.032 if (occupied_by is not None and occupied_by != object_name) else 0.0
        target_x = zone_pos[0]
        target_y = zone_pos[1] + offset_y

        # 1. Tiep can tren khong cua Zone (Di chuyen duong thang Cartesian ngang)
        zone_approach_pose = Pose()
        zone_approach_pose.position.x = target_x
        zone_approach_pose.position.y = target_y
        zone_approach_pose.position.z = self.approach_height
        zone_approach_pose.orientation = self.top_down_quaternion

        if not self._move_cartesian([zone_approach_pose]):
            if not self._move_to_pose_target(zone_approach_pose):
                return "PLANNING_FAILED"

        self._set_gazebo_model_pose(object_name, target_x, target_y, self.approach_height)
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

        # 3. Mo kep (Nha vat & dat vat on dinh len mat Zone)
        self.open_gripper()
        final_pos = [target_x, target_y, 0.02]
        self.object_positions[object_name] = final_pos
        self.zone_occupants[zone_name] = object_name
        self._set_gazebo_model_pose(object_name, final_pos[0], final_pos[1], final_pos[2])
        self._publish_dynamic_cube_state()

        # 4. Nhac kep len cao (Cartesian Retract)
        if not self._move_cartesian([zone_approach_pose]):
            self._move_to_pose_target(zone_approach_pose)

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

        # 1. Chuyen A vao zone_temp
        if self.pick(object_a) != "SUCCESS":
            return "FAILED"
        if self.place(object_a, "zone_temp") != "SUCCESS":
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
        self.open_gripper()
        final_pos = [pos_bottom[0], pos_bottom[1], 0.06]
        self.object_positions[object_top] = final_pos
        self._set_gazebo_model_pose(object_top, final_pos[0], final_pos[1], final_pos[2])
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

        # 4. Mo kep de nha vat
        self.open_gripper()
        self._set_gazebo_model_pose(object_name, init_pos[0], init_pos[1], init_pos[2])
        self.object_positions[object_name] = list(init_pos)
        self._publish_dynamic_cube_state()

        # 5. Nhac kep len cao
        if not self._move_cartesian([tray_approach]):
            self._move_to_pose_target(tray_approach)

        return "SUCCESS"

    def clear_zone(self, zone_name: str) -> str:
        """Gap vat the dang nam trong 1 zone ra ngoai va tra ve khay chua ban dau."""
        self.node.get_logger().info(f"Thuc thi Skill: clear_zone({zone_name})")
        occupant = self.zone_occupants.get(zone_name)
        if occupant is None:
            self.node.get_logger().info(f"Vung '{zone_name}' da trong, khong can don.")
            return "SUCCESS"

        self.node.get_logger().info(f"Dang gap '{occupant}' ra khoi '{zone_name}' de tra ve khay...")
        res = self._return_object_to_tray(occupant)
        if res == "SUCCESS":
            self.zone_occupants[zone_name] = None
        return res

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
        """
        Skill nang cao: Tra toan bo 3 khoi hop ve 3 khay phoi ban dau tren ban.
        """
        self.node.get_logger().info("Thuc thi Skill nang cao: reset_scene()")
        for name, data in self.objects_config.items():
            init_pos = data.get("initial_position", [0.24, 0.0, 0.02])
            self.object_positions[name] = list(init_pos)
            self._set_gazebo_model_pose(name, init_pos[0], init_pos[1], init_pos[2])

        for z in self.zone_occupants:
            self.zone_occupants[z] = None

        self._publish_dynamic_cube_state()
        self.home()
        return "SUCCESS"

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
            "positions": self.object_positions
        }

    def inspect_scene(self) -> dict:
        """Bao cao vi tri tat ca cac vat the, trang thai zone va trang thai tay kep."""
        return self.get_scene_state()

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
        """Lap ke hoach va thuc thi dich khop (Joint Target) voi goc ngan nhat khong quay xoay."""
        if not self._move_group_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("MoveGroup action server khong phan hoi!")
            return False

        unwrapped_targets = self._unwrap_joint_target(joint_values)

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.8
        goal_msg.request.max_acceleration_scaling_factor = 0.7
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
        self._wait_for_future(get_result_future, timeout_sec=30.0)

        result = get_result_future.result()
        return bool(result and result.result.error_code.val == 1)

    def _move_to_pose_target(self, target_pose: Pose) -> bool:
        """Lap ke hoach va thuc thi vi tri khong gian Descartes Pose voi rang buoc huong nghiem ngat."""
        if not self._move_group_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("MoveGroup action server khong phan hoi!")
            return False

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        goal_msg.request.num_planning_attempts = 15
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.8
        goal_msg.request.max_acceleration_scaling_factor = 0.7
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.request.start_state.is_diff = True

        # Dinh vi Constraint cho end-effector tool0
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.header.stamp = self.node.get_clock().now().to_msg()
        # Offset tool0 do co gan gripper (gripper dai ~8cm)
        pose_stamped.pose = Pose()
        pose_stamped.pose.position.x = target_pose.position.x
        pose_stamped.pose.position.y = target_pose.position.y
        pose_stamped.pose.position.z = target_pose.position.z + 0.08
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
        sp.dimensions = [0.03]
        bv.primitives.append(sp)
        bv.primitive_poses.append(pose_stamped.pose)
        pos_constraint.constraint_region = bv
        pos_constraint.weight = 1.0

        # Rang buoc huong kẹp: KHONG CHO PHEP quay loan xa quanh truc Z
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = "base_link"
        orient_constraint.link_name = "tool0"
        orient_constraint.orientation = target_pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.20
        orient_constraint.absolute_y_axis_tolerance = 0.20
        orient_constraint.absolute_z_axis_tolerance = 0.25  # Chat che, chan quay xoay bat thuong
        orient_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(orient_constraint)

        # Rang buoc khop co tay wrist_2 va khop vai shoulder_lift giu dang robot chuan (elbow-up)
        jc_w2 = JointConstraint()
        jc_w2.joint_name = "wrist_2_joint"
        w2_current = self.current_joint_positions.get("wrist_2_joint", -1.57)
        w2_diff = (-1.5708 - w2_current + math.pi) % (2.0 * math.pi) - math.pi
        jc_w2.position = w2_current + w2_diff
        jc_w2.tolerance_above = 0.8
        jc_w2.tolerance_below = 0.8
        jc_w2.weight = 0.8
        constraints.joint_constraints.append(jc_w2)

        goal_msg.request.goal_constraints.append(constraints)

        send_goal_future = self._move_group_client.send_goal_async(goal_msg)
        self._wait_for_future(send_goal_future, timeout_sec=10.0)

        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            return False

        get_result_future = goal_handle.get_result_async()
        self._wait_for_future(get_result_future, timeout_sec=30.0)

        result = get_result_future.result()
        return bool(result and result.result.error_code.val == 1)

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
            p.position.z = wp.position.z + 0.08
            p.orientation = wp.orientation
            offset_wps.append(p)

        req.waypoints = offset_wps
        req.max_step = 0.008
        req.jump_threshold = 0.0
        req.avoid_collisions = False

        future = self._cartesian_path_client.call_async(req)
        self._wait_for_future(future, timeout_sec=5.0)

        res = future.result()
        if not res or res.fraction < 0.75:
            return False

        # Thiet lap timestamp va van toc cho trajectory de joint_trajectory_controller chay em ai va nhanh
        traj = res.solution.joint_trajectory
        num_points = len(traj.points)
        if num_points == 0:
            return False

        current_time = 0.0
        traj.points[0].time_from_start.sec = 0
        traj.points[0].time_from_start.nanosec = 0
        traj.points[0].velocities = [0.0] * len(traj.points[0].positions)
        max_vel = self.cartesian_max_vel  # 1.2 rad/s (nhanh va dut khoat)

        for i in range(1, num_points):
            dt = 0.015
            for j in range(len(traj.points[i].positions)):
                dq = abs(traj.points[i].positions[j] - traj.points[i-1].positions[j])
                t_j = dq / max_vel
                if t_j > dt:
                    dt = t_j
            current_time += dt
            traj.points[i].time_from_start.sec = int(current_time)
            traj.points[i].time_from_start.nanosec = int((current_time % 1.0) * 1e9)

            traj.points[i].velocities = [
                (traj.points[i].positions[k] - traj.points[i-1].positions[k]) / max(dt, 0.001)
                for k in range(len(traj.points[i].positions))
            ]

        # Diem cuoi cung dung yen
        traj.points[-1].velocities = [0.0] * len(traj.points[-1].positions)

        # Thuc thi trajectory bang MoveIt ExecuteTrajectory (yeu cau RobotTrajectory)
        if self._execute_traj_client.wait_for_server(timeout_sec=3.0):
            goal = ExecuteTrajectory.Goal()
            goal.trajectory = res.solution
            exec_future = self._execute_traj_client.send_goal_async(goal)
            self._wait_for_future(exec_future, timeout_sec=10.0)
            handle = exec_future.result()
            if handle and handle.accepted:
                res_future = handle.get_result_async()
                self._wait_for_future(res_future, timeout_sec=30.0)
                exec_res = res_future.result()
                if exec_res:
                    err = getattr(getattr(exec_res, "result", None), "error_code", None)
                    err_val = getattr(err, "val", 0) if err else 0
                    status = getattr(exec_res, "status", 0)
                    return bool(err_val == 1 or status == 4)
        return False

    def _attach_object_to_robot(self, object_name: str):
        """Dinh kem vat vao tool0 trong PlanningScene de tranh va cham ao."""
        if not self._planning_scene_client.wait_for_service(timeout_sec=2.0):
            return

        ps = PlanningScene()
        ps.is_diff = True

        attached_obj = AttachedCollisionObject()
        attached_obj.link_name = "tool0"
        attached_obj.object.id = object_name
        attached_obj.object.operation = CollisionObject.ADD
        attached_obj.touch_links = ["tool0", "gripper_base", "gripper_left_finger", "gripper_right_finger"]

        ps.robot_state.attached_collision_objects.append(attached_obj)

        req = ApplyPlanningScene.Request()
        req.scene = ps
        self._planning_scene_client.call_async(req)

    def _detach_object_from_robot(self, object_name: str):
        """Go vat the khoi tool0 trong PlanningScene."""
        if not self._planning_scene_client.wait_for_service(timeout_sec=2.0):
            return

        ps = PlanningScene()
        ps.is_diff = True

        attached_obj = AttachedCollisionObject()
        attached_obj.link_name = "tool0"
        attached_obj.object.id = object_name
        attached_obj.object.operation = CollisionObject.REMOVE

        ps.robot_state.attached_collision_objects.append(attached_obj)

        req = ApplyPlanningScene.Request()
        req.scene = ps
        self._planning_scene_client.call_async(req)
