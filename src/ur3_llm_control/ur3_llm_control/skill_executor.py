#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Skill Executor:
Dieu phoi thuc thi ke hoach cac Robot Skill theo dung trinh tu.
In format dau ra Terminal chuan chi tiet theo dung yeu cau de bai:

USER COMMAND:
...
LLM PLAN:
...
EXECUTION:
pick(...) ........ SUCCESS
place(...) ....... SUCCESS
home() ........... SUCCESS

TASK SUCCESS
"""

import time
from typing import List, Dict, Any
from .robot_skills import RobotSkills


class SkillExecutor:
    """Dieu phoi thuc thi tung Robot Skill va in log chuan hoa."""

    def __init__(self, robot_skills: RobotSkills):
        self.skills = robot_skills

    def format_skill_call(self, step: Dict[str, Any]) -> str:
        """Dinh dang ten ham skill voi cac tham so."""
        skill = step.get("skill", "")
        if skill == "home":
            return "home()"
        elif skill == "pick":
            return f"pick({step.get('object', '')})"
        elif skill == "place":
            return f"place({step.get('object', '')}, {step.get('zone', '')})"
        elif skill == "move_above":
            target = step.get("object") or step.get("zone", "")
            return f"move_above({target})"
        elif skill == "move_to_zone":
            return f"move_to_zone({step.get('zone', '')})"
        elif skill == "open_gripper":
            return "open_gripper()"
        elif skill == "close_gripper":
            obj = step.get("object")
            return f"close_gripper({obj})" if obj else "close_gripper()"
        return f"{skill}()"

    def execute_plan(self, user_command: str, plan_data: Dict[str, Any], source_info: str = "LLM") -> bool:
        """
        Thuc thi chuoi cac action trong plan_data.
        
        Returns:
            True neu toan bo plan thanh cong, False neu co bat ky buoc nao that bai.
        """
        plan: List[Dict[str, Any]] = plan_data.get("plan", [])
        thought: str = plan_data.get("thought", "")

        print("\n" + "=" * 65)
        print("USER COMMAND:")
        print(f"  {user_command}")
        print("-" * 65)

        if thought:
            print(f"LLM REASONING ({source_info}):")
            print(f"  {thought}")
            print("-" * 65)

        print("LLM PLAN:")
        for step in plan:
            print(f"    {self.format_skill_call(step)}")
        print("-" * 65)

        print("EXECUTION:")
        all_success = True

        for step in plan:
            skill_name = step.get("skill", "").strip().lower()
            call_repr = self.format_skill_call(step)
            dots = "." * max(2, 28 - len(call_repr))

            status = "FAILED"
            try:
                if skill_name == "home":
                    status = self.skills.home()
                elif skill_name == "pick":
                    status = self.skills.pick(step.get("object"))
                elif skill_name == "place":
                    status = self.skills.place(step.get("object"), step.get("zone"))
                elif skill_name == "move_above":
                    target = step.get("object") or step.get("zone")
                    status = self.skills.move_above(target)
                elif skill_name == "move_to_zone":
                    status = self.skills.move_to_zone(step.get("zone"))
                elif skill_name == "open_gripper":
                    status = self.skills.open_gripper()
                elif skill_name == "close_gripper":
                    status = self.skills.close_gripper(step.get("object"))
                else:
                    status = "INVALID_SKILL"
            except Exception as e:
                status = f"EXCEPTION ({e})"

            print(f"{call_repr} {dots} {status}")

            if status != "SUCCESS":
                all_success = False
                print(f"[ERROR] Dung thuc thi tai buoc: {call_repr} voi trang thai '{status}'")
                break

        print("-" * 65)
        if all_success:
            print("TASK SUCCESS")
        else:
            print("TASK FAILED")
        print("=" * 65 + "\n")

        return all_success
