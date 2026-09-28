#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
User Command Console:
Cung cap giao dien dong lenh tuong tac (Interactive Terminal CLI) rieng biet.
Nguoi dung co the go cau lenh bang tieng Viet hoac tieng Anh truc tiep,
gui toi Robot thong qua topic /user_command va nhan phan hoi tuc thi.
"""

import sys
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


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
        print(f"\n[FEEDBACK] {msg.data}\n[UR3-LLM] Nhap cau lenh > ", end="", flush=True)

    def send_command(self, cmd: str):
        """Gui cau lenh toi robot qua topic /user_command."""
        msg = String()
        msg.data = cmd
        self.pub_cmd.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UserConsole()

    # Chay spin trong thread rieng de khong can tro vong lap input()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("\n" + "=" * 68)
    print("🤖 GIAO DIEN NHAP LENH ROBOT UR3 (LLM & SKILL-BASED PLANNING)")
    print("Sinh vien: Le Anh Tuan Bang | MSSV: 23020723 | P = 5")
    print("Quy uoc Zone: Zone A -> Blue | Zone B -> Yellow | Zone C -> Red")
    print("=" * 68)
    print("Vi du cau lenh co ban (1 vat the):")
    print("  - Dua khoi mau do vao vung B       (Put the red cube in zone B)")
    print("  - Chuyen khoi mau vang sang vung C  (Move yellow cube to zone C)")
    print("  - Dat khoi xanh lam vao o A         (Place blue cube in zone A)")
    print("Vi du cau lenh nang cao (Theo MSSV 23020723):")
    print("  - Hay sap xep tat ca cac vat the theo ma sinh vien cua toi")
    print("  - Arrange all objects according to my student ID")
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
                print(f"-> [DA GUI] Lenh: '{cmd}'")
                print("   Dang cho Robot lap ke hoach va thuc thi...")
            except EOFError:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
