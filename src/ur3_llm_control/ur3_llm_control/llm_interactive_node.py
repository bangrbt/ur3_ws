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
        super().__init__("llm_interactive_node", automatically_declare_parameters_from_overrides=True)
        self.get_logger().info("Khoi tao LLM Interactive Node...")

        # Khoa tranh chay dong thoi nhieu lenh cung luc
        self.execution_lock = threading.Lock()

        # Khai bao parameters (neu chua co trong overrides)
        if not self.has_parameter("command"):
            self.declare_parameter("command", "")
        if not self.has_parameter("interactive"):
            self.declare_parameter("interactive", True)
        if not self.has_parameter("config_dir"):
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
        self.skill_executor = SkillExecutor(self.skills, feedback_cb=self._send_feedback)

        # 3. Subscriber nhan cau lenh qua topic /user_command & Publisher phan hoi
        self.sub_cmd = self.create_subscription(
            String, "/user_command", self._topic_command_callback, 10
        )
        self.feedback_pub = self.create_publisher(String, "/command_feedback", 10)

        self.get_logger().info("=" * 65)
        self.get_logger().info(f"Sinh vien: {self.planner.student_name} - MSSV: {self.planner.student_id}")
        self.get_logger().info(f"Quy uoc P = {self.planner.p_value}: Zone A -> {self.planner.zone_mapping['zone_a']}, Zone B -> {self.planner.zone_mapping['zone_b']}, Zone C -> {self.planner.zone_mapping['zone_c']}")
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

    def _send_feedback(self, text: str):
        """Gui thong diep phan hoi qua topic /command_feedback."""
        try:
            msg = String()
            msg.data = text
            self.feedback_pub.publish(msg)
        except Exception:
            pass

    def _topic_command_callback(self, msg: String):
        """Callback khi nhan cau lenh qua topic /user_command."""
        cmd = msg.data.strip()
        if not cmd:
            return
        if self.execution_lock.locked():
            self.get_logger().warn(f"Robot dang ban! Tu choi nhan lenh moi: '{cmd}'")
            self._send_feedback("\n[ROBOT DANG BAN] Robot dang thuc thi cau lenh truoc do. Vui long cho hoan tat roi thu lai!")
            return
        self.get_logger().info(f"Nhan cau lenh qua /user_command: '{cmd}'")
        threading.Thread(target=self.process_command, args=(cmd,), daemon=True).start()

    def _console_loop(self):
        """Vong lap doc cau lenh truc tiep tu ban phim console."""
        print("\n" + "=" * 65, flush=True)
        print("  HE THONG DIEU KHIEN ROBOT UR3 BANG LLM & SKILL-BASED PLANNING", flush=True)
        print(f"  Sinh vien: {self.planner.student_name} - MSSV: {self.planner.student_id}", flush=True)
        print("=" * 65, flush=True)
        print("Vi du cau lenh co ban:", flush=True)
        print("  - 'Put the red cube in zone B' hoac 'Đưa khối màu đỏ vào vùng B'", flush=True)
        print("  - 'Move the blue cube to zone A' hoac 'Chuyển khối màu xanh lam sang ô A'", flush=True)
        print("  - 'Hãy lấy khối màu vàng và đặt nó vào ô C'", flush=True)
        print("Vi du cau lenh nang cao (Ca nhan hoa theo MSSV):", flush=True)
        print("  - 'Arrange all objects according to my student ID'", flush=True)
        print("  - 'Hãy sắp xếp các khối theo mã sinh viên của tôi'", flush=True)
        print("=" * 65, flush=True)

        while rclpy.ok():
            try:
                user_input = input("\n[UR3-LLM] Nhap cau lenh (hoac 'exit' de thoat) > ").strip()
                if not user_input:
                    continue
                if user_input.lower() in ["exit", "quit"]:
                    print("Dang thoat...", flush=True)
                    os._exit(0)

                self.process_command(user_input)
            except (EOFError, KeyboardInterrupt):
                break

    def process_command(self, command: str):
        """Xu ly toan ven chu trinh: Command -> LLM -> Validator -> Executor."""
        if not self.execution_lock.acquire(blocking=False):
            self.get_logger().warn(f"Robot dang ban! Khong the xu ly: '{command}'")
            self._send_feedback("\n[ROBOT DANG BAN] Robot dang thuc thi. Vui long doi!")
            return

        try:
            msg_start = f"Dang xu ly cau lenh: '{command}'..."
            self.get_logger().info(msg_start)
            self._send_feedback(msg_start)

            # 1. LLM Task Planner sinh ke hoach co cau truc
            plan_dict, source_info = self.planner.plan(command)

            # 2. Plan Validator kiem tra tinh hop le
            current_holding = self.skills.holding_object
            is_valid, validation_msg = self.validator.validate_plan(plan_dict, initial_holding=current_holding)

            if not is_valid:
                err_msg = (
                    "\n" + "!" * 65 + "\n"
                    + "[PLAN VALIDATOR] KE HOACH BI TU CHOI THUC THI!\n"
                    + f"Ly do: {validation_msg}\n"
                    + "!" * 65 + "\n"
                )
                self.get_logger().warn(f"Ke hoach bi tu choi: {validation_msg}")
                self._send_feedback(err_msg)
                return

            self.get_logger().info(f"[PLAN VALIDATOR] PASSED: {validation_msg}")

            # 3. Skill Executor thuc thi tung Robot Skill
            self.skill_executor.execute_plan(command, plan_dict, source_info=source_info)
        finally:
            self.execution_lock.release()


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
