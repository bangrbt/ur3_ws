#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Skills Library:
Tap hop cac ky nang nguyen thuy (Primitive Skills) dieu khien robot UR3/UR3e
thong qua MoveIt 2.

Cac skill toi thieu:
- home()
- pick(object)
- place(object, zone)
Cac skill bo sung:
- move_above(object)
- open_gripper()
- close_gripper(object)
- move_to_zone(zone)
- inspect_state()

Moi skill tra ve trang thai thuc thi: SUCCESS, FAILED, INVALID_OBJECT, PLANNING_FAILED
"""

import time
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import (
    PlanningScene,
    CollisionObject,
    AttachedCollisionObject,
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath, ApplyPlanningScene
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class RobotSkills:
    """Thu vien ky nang MoveIt 2 cho canh tay UR3."""

    def __init__(self, node: Node, scene_config: dict):
        self.node = node
        self.scene_config = scene_config

        self.table_config = scene_config.get("table", {})
        self.objects_config = scene_config.get("objects", {})
        self.zones_config = scene_config.get("zones", {})
        self.motion_params = scene_config.get("motion_params", {})

        # Tọa độ vị trí các vật thể hiện tại (được cập nhật động khi pick/place)
        self.object_positions = {}
        for obj_name, obj_data in self.objects_config.items():
            self.object_positions[obj_name] = list(obj_data.get("initial_position", [0.3, 0.0, 0.02]))

        self.holding_object = None  # Vat the robot dang kep

        # Action Clients MoveIt 2
        self._move_group_client = ActionClient(self.node, MoveGroup, "/move_action")
        self._execute_traj_client = ActionClient(self.node, ExecuteTrajectory, "/execute_trajectory")
        self._cartesian_path_client = self.node.create_client(GetCartesianPath, "/compute_cartesian_path")
        self._planning_scene_client = self.node.create_client(ApplyPlanningScene, "/apply_planning_scene")

        # Orientation chúi thẳng đứng vuông góc mặt bàn (Top-down grasp)
        # Khâu tác động cuối quay hướng xuống bàn: Pitch = 180 độ
        self.top_down_quaternion = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)

        # Tham số chiều cao
        self.approach_height = self.motion_params.get("approach_height", 0.14)
        self.grasp_height = self.motion_params.get("grasp_height", 0.04)
        self.place_height = self.motion_params.get("place_height", 0.04)
        self.home_joints = self.motion_params.get("home_joints", [0.0, -1.3, 1.5, -1.7, -1.57, 0.0])

        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

    # =========================================================================
    # --- CAC SKILL NGUYEN THUY ---
    # =========================================================================

    def home(self) -> str:
        """Dua tay may ve tu the cho mac dinh (Home pose)."""
        self.node.get_logger().info("Thuc thi Skill: home()")
        success = self._move_to_joint_target(self.home_joints)
        return "SUCCESS" if success else "PLANNING_FAILED"

    def open_gripper(self) -> str:
        """Mo kep va nha vat the khoi PlanningScene (neu dang giu)."""
        self.node.get_logger().info("Thuc thi Skill: open_gripper()")
        if self.holding_object:
            self._detach_object_from_robot(self.holding_object)
            self.holding_object = None
        time.sleep(0.5)
        return "SUCCESS"

    def close_gripper(self, object_name: str = None) -> str:
        """Dong kep va dinh kem vat the vao PlanningScene robot."""
        self.node.get_logger().info(f"Thuc thi Skill: close_gripper(object={object_name})")
        if object_name:
            self._attach_object_to_robot(object_name)
            self.holding_object = object_name
        time.sleep(0.5)
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

        success = self._move_to_pose_target(target_pose)
        return "SUCCESS" if success else "PLANNING_FAILED"

    def move_to_zone(self, zone_name: str) -> str:
        """Di chuyen tay kep den phia tren vung Zone."""
        self.node.get_logger().info(f"Thuc thi Skill: move_to_zone({zone_name})")
        return self.move_above(zone_name)

    def pick(self, object_name: str) -> str:
        """
        Chu trinh gap vat hoan chinh:
        1. Kiem tra tinh hop le
        2. Mo kep
        3. Tiep can tren khong (Move Above)
        4. Ha but cham vat (Cartesian Descend)
        5. Dong kep (Attach object)
        6. Nhac vat len cao (Cartesian Lift)
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

        # 1. Mo kep
        self.open_gripper()

        # 2. Tiep can tren khong
        approach_pose = Pose()
        approach_pose.position.x = obj_xy[0]
        approach_pose.position.y = obj_xy[1]
        approach_pose.position.z = self.approach_height
        approach_pose.orientation = self.top_down_quaternion

        if not self._move_to_pose_target(approach_pose):
            return "PLANNING_FAILED"

        # 3. Ha kẹp Cartesian xuong vat
        grasp_pose = Pose()
        grasp_pose.position.x = obj_xy[0]
        grasp_pose.position.y = obj_xy[1]
        grasp_pose.position.z = self.grasp_height
        grasp_pose.orientation = self.top_down_quaternion

        if not self._move_cartesian([grasp_pose]):
            # Thu move thuong neu cartesian khong kha thi
            self._move_to_pose_target(grasp_pose)

        # 4. Dong kep (Dinh kem vat the)
        self.close_gripper(object_name)

        # 5. Nhac vat len cao
        if not self._move_cartesian([approach_pose]):
            self._move_to_pose_target(approach_pose)

        return "SUCCESS"

    def place(self, object_name: str, zone_name: str) -> str:
        """
        Chu trinh dat vat hoan chinh:
        1. Kiem tra tinh hop le
        2. Di chuyen tren khong toi Zone
        3. Ha vat xuong mat ban (Cartesian Descend)
        4. Mo kep (Detach object)
        5. Nhac kep len cao (Cartesian Retract)
        6. Cap nhat toa do vat the ve Zone
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
        zone_pos = zone_data.get("position", [0.4, 0.0, 0.001])

        # 1. Tiep can tren khong cua Zone
        approach_pose = Pose()
        approach_pose.position.x = zone_pos[0]
        approach_pose.position.y = zone_pos[1]
        approach_pose.position.z = self.approach_height
        approach_pose.orientation = self.top_down_quaternion

        if not self._move_to_pose_target(approach_pose):
            return "PLANNING_FAILED"

        # 2. Ha dat vat xuong mat ban
        place_pose = Pose()
        place_pose.position.x = zone_pos[0]
        place_pose.position.y = zone_pos[1]
        place_pose.position.z = self.place_height
        place_pose.orientation = self.top_down_quaternion

        if not self._move_cartesian([place_pose]):
            self._move_to_pose_target(place_pose)

        # 3. Mo kep (Nha vat)
        self.open_gripper()

        # 4. Nhac kep len cao
        if not self._move_cartesian([approach_pose]):
            self._move_to_pose_target(approach_pose)

        # 5. Cap nhat toa do vat the moi
        self.object_positions[object_name] = [zone_pos[0], zone_pos[1], 0.02]

        return "SUCCESS"

    def inspect_state(self) -> dict:
        """Bao cao vi tri tat ca cac vat the va trang thai tay kep."""
        return {
            "holding": self.holding_object,
            "positions": self.object_positions
        }

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
            time.sleep(0.02)
        return future.done()

    def _move_to_joint_target(self, joint_values: list) -> bool:
        """Lap ke hoach va thuc thi dich khop (Joint Target)."""
        if not self._move_group_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("MoveGroup action server khong phan hoi!")
            return False

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.request.start_state.is_diff = True

        constraints = Constraints()
        for idx, (name, val) in enumerate(zip(self.joint_names, joint_values)):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(val)
            jc.tolerance_above = 0.02
            jc.tolerance_below = 0.02
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
        if result and result.result.error_code.val == 1:
            return True
        return False

    def _move_to_pose_target(self, target_pose: Pose) -> bool:
        """Lap ke hoach va thuc thi vi tri khong gian Descartes Pose."""
        if not self._move_group_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error("MoveGroup action server khong phan hoi!")
            return False

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.request.start_state.is_diff = True

        # Dinh vi Constraint cho end-effector tool0
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.header.stamp = rclpy.time.Time().to_msg()
        # Offset tool0 do co gan gripper (gripper dai ~8cm)
        pose_stamped.pose = target_pose
        pose_stamped.pose.position.z += 0.08

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
        sp.dimensions = [0.025]
        bv.primitives.append(sp)
        bv.primitive_poses.append(pose_stamped.pose)
        pos_constraint.constraint_region = bv
        pos_constraint.weight = 1.0

        # Rang buoc goc quay
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = "base_link"
        orient_constraint.link_name = "tool0"
        orient_constraint.orientation = target_pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.35
        orient_constraint.absolute_y_axis_tolerance = 0.35
        orient_constraint.absolute_z_axis_tolerance = 3.14159
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
        self._wait_for_future(get_result_future, timeout_sec=30.0)

        result = get_result_future.result()
        return bool(result and result.result.error_code.val == 1)

    def _move_cartesian(self, waypoints: list) -> bool:
        """Di chuyen duong thang Descartes mem mai."""
        if not self._cartesian_path_client.wait_for_service(timeout_sec=2.0):
            time.sleep(1.0)
            return True

        req = GetCartesianPath.Request()
        req.header.frame_id = "base_link"
        req.header.stamp = rclpy.time.Time().to_msg()
        req.group_name = "ur_manipulator"
        req.link_name = "tool0"

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
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.avoid_collisions = True

        future = self._cartesian_path_client.call_async(req)
        self._wait_for_future(future, timeout_sec=5.0)

        res = future.result()
        if res and res.fraction > 0.8:
            # Thuc thi trajectory
            if self._execute_traj_client.wait_for_server(timeout_sec=2.0):
                goal = ExecuteTrajectory.Goal()
                goal.trajectory = res.solution
                exec_future = self._execute_traj_client.send_goal_async(goal)
                self._wait_for_future(exec_future, timeout_sec=10.0)
                handle = exec_future.result()
                if handle and handle.accepted:
                    res_future = handle.get_result_async()
                    self._wait_for_future(res_future, timeout_sec=30.0)
                    return True
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
