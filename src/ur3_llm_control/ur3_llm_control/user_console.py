#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
User Command Console:
Cung cap giao dien dong lenh tuong tac (Interactive Terminal CLI) rieng biet.
Nguoi dung co the go cau lenh bang tieng Viet hoac tieng Anh truc tiep,
gui toi Robot thong qua topic /user_command va nhan phan hoi tuc thi.
Tu dong doc MSSV va tinh toan P = XX mod 6 theo thoi gian thuc.
"""

import os
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import sys
import yaml
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .student_utils import parse_student_info, get_color_name


class UserConsole(Node):
    """ROS 2 Node giao dien dong lenh tuong tac cho nguoi dung."""

    def __init__(self):
        super().__init__("user_console_node")
        self.pub_cmd = self.create_publisher(String, "/user_command", 10)
        self.sub_feedback = self.create_subscription(
            String, "/command_feedback", self._feedback_callback, 10
        )

    def _feedback_callback(self, msg: String):
        """Hien thi thong tin phan hoi tu robot."""
        print(f"{msg.data}", flush=True)
        if "TASK SUCCESS" in msg.data or "TASK FAILED" in msg.data or "TU CHOI THUC THI" in msg.data:
            print("\n[UR3-LLM] Nhap cau lenh (hoac 'exit' de thoat) > ", end="", flush=True)

    def send_command(self, cmd: str):
        """Gui cau lenh toi robot qua topic /user_command."""
        msg = String()
        msg.data = cmd
        self.pub_cmd.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UserConsole()

    # Doc thong tin sinh vien tu config de tu dong tinh P
    try:
        from ament_index_python.packages import get_package_share_directory
        cfg_file = os.path.join(get_package_share_directory("ur3_llm_control"), "config", "student_config.yaml")
    except Exception:
        cfg_file = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "student_config.yaml")

    student_name = "Lê Anh Tuấn Bằng"
    student_id = "23020723"
    if os.path.exists(cfg_file):
        with open(cfg_file, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
            student_name = data.get("student_name", student_name)
            student_id = str(data.get("student_id", student_id))

    xx, p, mapping = parse_student_info(student_id)

    # Chay spin trong thread rieng de khong can tro vong lap input()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("\n" + "=" * 68)
    print("🤖 GIAO DIEN NHAP LENH ROBOT UR3 (LLM & SKILL-BASED PLANNING)")
    print(f"Sinh vien: {student_name} | MSSV: {student_id} | XX: {xx} -> P = {p}")
    print(f"Quy uoc Zone: Zone A -> {get_color_name(mapping['zone_a'])} | Zone B -> {get_color_name(mapping['zone_b'])} | Zone C -> {get_color_name(mapping['zone_c'])}")
    print("=" * 68)
    print("Vi du cau lenh co ban (1 vat the):")
    print("  - Dua khoi mau do vao vung B        (Put the red cube in zone B)")
    print("  - Chuyen khoi mau vang sang vung C   (Move yellow cube to zone C)")
    print("  - Dat khoi xanh lam vao o A          (Place blue cube in zone A)")
    print(f"Vi du cau lenh ca nhan hoa (Theo MSSV {student_id}):")
    print("  - Hay sap xep tat ca cac vat the theo ma sinh vien cua toi")
    print("  - Arrange all objects according to my student ID")
    print("Vi du cau lenh nang cao:")
    print("  - Hay doi cho khoi do va khoi vang   (Swap red and yellow cube)")
    print("  - Xep khoi do len tren khoi vang     (Stack red on yellow cube)")
    print("  - Dat lai tat ca cac khoi ve ban dau (Reset scene to initial trays)")
    print("  - Kiem tra vi tri cac vat the        (Inspect workspace status)")
    print("Kiem tra tu choi lenh khong hop le:")
    print("  - Gap qua tao dat vao vung D")
    print("=" * 68 + "\n")

    try:
        while rclpy.ok():
            try:
                cmd = input("[UR3-LLM] Nhap cau lenh (hoac 'exit' de thoat) > ").strip()
                if not cmd:
                    continue
                if cmd.lower() in ["exit", "quit", "q"]:
                    print("Dang thoat console...")
                    break

                node.send_command(cmd)
                print(f"-> [DA GUI] Lenh: '{cmd}'", flush=True)
                print("   Dang cho Robot lap ke hoach va thuc thi...\n", flush=True)
            except EOFError:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
