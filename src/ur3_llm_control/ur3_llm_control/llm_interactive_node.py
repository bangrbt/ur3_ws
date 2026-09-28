#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Main Node: llm_interactive_node
Node trung tam tiep nhan cau lenh ngon ngu tu nhien (tu terminal, doi so launch hoac topic),
goi LLM Planner sinh ke hoach, kiem tra tinh hop le bang Task Validator,
va dieu phoi thuc thi bang Skill Executor.
"""

import sys
import os
import yaml
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .llm_planner import LLMPlanner
from .task_validator import TaskValidator
from .robot_skills import RobotSkills
from .skill_executor import SkillExecutor


class LLMInteractiveNode(Node):
    """ROS 2 Node tuong tac giua nguoi dung, LLM va Robot."""

    def __init__(self):
        super().__init__("llm_interactive_node")
        self.get_logger().info("Khoi tao LLM Interactive Node...")

        # Khai bao parameters
        self.declare_parameter("command", "")
        self.declare_parameter("interactive", True)
        self.declare_parameter("config_dir", "")

        config_dir_param = self.get_parameter("config_dir").get_parameter_value().string_value
        if not config_dir_param:
            try:
                from ament_index_python.packages import get_package_share_directory
                config_dir_param = os.path.join(get_package_share_directory("ur3_llm_control"), "config")
            except Exception:
                config_dir_param = os.path.join(
                    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config"
                )

        # 1. Load Cau hinh Scene
        scene_file = os.path.join(config_dir_param, "scene.yaml")
        with open(scene_file, "r", encoding="utf-8") as f:
            self.scene_config = yaml.safe_load(f)

        # 2. Khoi tao cac Module
        self.planner = LLMPlanner(config_dir=config_dir_param)
        self.validator = TaskValidator()
        self.skills = RobotSkills(self, self.scene_config)
        self.skill_executor = SkillExecutor(self.skills)

        # 3. Subscriber nhan cau lenh qua topic /user_command & Publisher phan hoi
        self.sub_cmd = self.create_subscription(
            String, "/user_command", self._topic_command_callback, 10
        )
        self.feedback_pub = self.create_publisher(String, "/command_feedback", 10)

        self.get_logger().info("=" * 65)
        self.get_logger().info(f"Sinh vien: {self.planner.student_name} - MSSV: {self.planner.student_id}")
        self.get_logger().info(f"Quy uoc P = {self.planner.p_value}: Zone A -> Blue, Zone B -> Yellow, Zone C -> Red")
        self.get_logger().info("LLM Interactive Node da san sang!")
        self.get_logger().info("=" * 65)

        # 4. Kiem tra xem co cau lenh ban dau truyen qua parameter khong
        initial_cmd = self.get_parameter("command").get_parameter_value().string_value
        if initial_cmd:
            self.get_logger().info(f"Phat hien parameter command: '{initial_cmd}'")
            # Chay trong thread rieng de khong block ROS spin
            threading.Thread(target=self.process_command, args=(initial_cmd,), daemon=True).start()

        # 5. Che do nhap ban phim Console Interactive
        is_interactive = self.get_parameter("interactive").get_parameter_value().bool_value
        if is_interactive and not initial_cmd:
            threading.Thread(target=self._console_loop, daemon=True).start()

    def _topic_command_callback(self, msg: String):
        """Callback khi nhan cau lenh qua topic /user_command."""
        cmd = msg.data.strip()
        if cmd:
            self.get_logger().info(f"Nhan cau lenh qua /user_command: '{cmd}'")
            threading.Thread(target=self.process_command, args=(cmd,), daemon=True).start()

    def _console_loop(self):
        """Vong lap doc cau lenh truc tiep tu ban phim console."""
        print("\n" + "=" * 65)
        print("  HE THONG DIEU KHIEN ROBOT UR3 BANG LLM & SKILL-BASED PLANNING")
        print(f"  Sinh vien: {self.planner.student_name} - MSSV: {self.planner.student_id}")
        print("=" * 65)
        print("Vi du cau lenh co ban:")
        print("  - 'Put the red cube in zone B' hoac 'Đưa khối màu đỏ vào vùng B'")
        print("  - 'Move the blue cube to zone A' hoac 'Chuyển khối màu xanh lam sang ô A'")
        print("  - 'Hãy lấy khối màu vàng và đặt nó vào ô C'")
        print("Vi du cau lenh nang cao (Ca nhan hoa theo MSSV):")
        print("  - 'Arrange all objects according to my student ID'")
        print("  - 'Hãy sắp xếp các khối theo mã sinh viên của tôi'")
        print("=" * 65)

        while rclpy.ok():
            try:
                user_input = input("\n[UR3-LLM] Nhap cau lenh (hoac 'exit' de thoat) > ").strip()
                if not user_input:
                    continue
                if user_input.lower() in ["exit", "quit"]:
                    print("Dang thoat...")
                    os._exit(0)

                self.process_command(user_input)
            except (EOFError, KeyboardInterrupt):
                break

    def process_command(self, command: str):
        """Xu ly toan ven chu trinh: Command -> LLM -> Validator -> Executor."""
        print(f"\n[INFO] Dang xu ly cau lenh: '{command}'...")

        # 1. LLM Task Planner sinh ke hoach co cau truc
        plan_dict, source_info = self.planner.plan(command)

        # 2. Plan Validator kiem tra tinh hop le
        current_holding = self.skills.holding_object
        is_valid, validation_msg = self.validator.validate_plan(plan_dict, initial_holding=current_holding)

        if not is_valid:
            print("\n" + "!" * 65)
            print("[PLAN VALIDATOR] KE HOACH BI TU CHOI THUC THI!")
            print(f"Ly do: {validation_msg}")
            print(f"Raw Plan: {plan_dict}")
            print("!" * 65 + "\n")
            return

        print(f"[PLAN VALIDATOR] PASSED: {validation_msg}")

        # 3. Skill Executor thuc thi tung Robot Skill
        self.skill_executor.execute_plan(command, plan_dict, source_info=source_info)


def main(args=None):
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = LLMInteractiveNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
