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

from __future__ import annotations
import time
from typing import List, Dict, Any, TYPE_CHECKING

if TYPE_CHECKING:
    from .robot_skills import RobotSkills


class SkillExecutor:
    """Dieu phoi thuc thi tung Robot Skill va in log chuan hoa."""

    def __init__(self, robot_skills: RobotSkills, feedback_cb=None):
        self.skills = robot_skills
        self.feedback_cb = feedback_cb

    def _log(self, text: str):
        print(text, flush=True)
        if self.feedback_cb:
            try:
                self.feedback_cb(text)
            except Exception:
                pass

    def format_skill_call(self, step: Dict[str, Any]) -> str:
        """Dinh dang ten ham skill voi cac tham so."""
        skill = step.get("skill", "")
        if skill == "home":
            return "home()"
        elif skill == "pick":
            return f"pick({step.get('object', '')})"
        elif skill == "place":
            return f"place({step.get('object', '')}, {step.get('zone', '')})"
        elif skill == "clear_zones":
            return "clear_zones()"
        elif skill == "clear_zone":
            return f"clear_zone({step.get('zone', '')})"
        elif skill == "swap":
            obj_a = step.get("object_a") or step.get("object1", "")
            obj_b = step.get("object_b") or step.get("object2", "")
            return f"swap({obj_a}, {obj_b})"
        elif skill == "stack":
            top = step.get("object_top") or step.get("top", "")
            bottom = step.get("object_bottom") or step.get("bottom", "")
            return f"stack({top}, {bottom})"
        elif skill == "reset_scene":
            return "reset_scene()"
        elif skill == "inspect_scene":
            return "inspect_scene()"
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
        elif skill == "detect_objects":
            return "detect_objects()"
        elif skill == "check_zone":
            return f"check_zone({step.get('zone', '')})"
        elif skill == "find_free_position":
            return "find_free_position()"
        return f"{skill}()"

    def execute_plan(self, user_command: str, plan_data: Dict[str, Any], source_info: str = "LLM", connection_alert: str = None) -> bool:
        """
        Thuc thi chuoi cac action trong plan_data.
        
        Returns:
            True neu toan bo plan thanh cong, False neu co bat ky buoc nao that bai.
        """
        plan: List[Dict[str, Any]] = plan_data.get("plan", [])
        thought: str = plan_data.get("thought", "")

        self._log("\n" + "=" * 65)
        if connection_alert:
            self._log(connection_alert)
            self._log("-" * 65)

        self._log("USER COMMAND:")
        self._log(f"  {user_command}")
        self._log("-" * 65)

        if thought:
            self._log(f"LLM REASONING ({source_info}):")
            self._log(f"  {thought}")
            self._log("-" * 65)

        self._log("LLM PLAN:")
        for step in plan:
            self._log(f"    {self.format_skill_call(step)}")
        self._log("-" * 65)

        self._log("EXECUTION:")
        all_success = True

        for idx, step in enumerate(plan):
            skill_name = step.get("skill", "").strip().lower()
            call_repr = self.format_skill_call(step)
            dots = "." * max(2, 28 - len(call_repr))

            # Kiem tra vat the da o san vi tri dich (bo qua khong gap len roi tha lai)
            if skill_name == "pick":
                obj = step.get("object")
                target_zone = None
                for next_idx in range(idx + 1, len(plan)):
                    next_s = plan[next_idx]
                    if next_s.get("skill", "").strip().lower() == "place":
                        if (next_s.get("object") or obj) == obj:
                            target_zone = next_s.get("zone")
                            break
                    elif next_s.get("skill", "").strip().lower() == "pick":
                        break

                if target_zone:
                    cur_loc = self.skills.get_cube_locations().get(obj)
                    is_occupied = (self.skills.zone_occupants.get(target_zone) == obj)
                    if cur_loc == target_zone or is_occupied:
                        self._log(f"{call_repr} {dots} SKIPPED (Vật đã ở sẵn vị trí '{target_zone}')")
                        continue

            elif skill_name == "place":
                obj = step.get("object")
                target_zone = step.get("zone")
                cur_loc = self.skills.get_cube_locations().get(obj)
                is_occupied = (self.skills.zone_occupants.get(target_zone) == obj)
                if self.skills.holding_object is None and (cur_loc == target_zone or is_occupied):
                    self._log(f"{call_repr} {dots} SKIPPED (Vật đã ở sẵn vị trí '{target_zone}')")
                    continue

            status = "FAILED"
            try:
                if skill_name == "home":
                    status = self.skills.home()
                elif skill_name == "pick":
                    status = self.skills.pick(step.get("object"))
                elif skill_name == "place":
                    status = self.skills.place(step.get("object"), step.get("zone"))
                elif skill_name == "clear_zones":
                    status = self.skills.clear_zones()
                elif skill_name == "clear_zone":
                    status = self.skills.clear_zone(step.get("zone"))
                elif skill_name == "swap":
                    obj_a = step.get("object_a") or step.get("object1")
                    obj_b = step.get("object_b") or step.get("object2")
                    status = self.skills.swap(obj_a, obj_b)
                elif skill_name == "stack":
                    top = step.get("object_top") or step.get("top")
                    bottom = step.get("object_bottom") or step.get("bottom")
                    status = self.skills.stack(top, bottom)
                elif skill_name == "reset_scene":
                    status = self.skills.reset_scene()
                elif skill_name == "inspect_scene":
                    res = self.skills.inspect_scene()
                    self._log(f"    Trang thai: {res}")
                    status = "SUCCESS"
                elif skill_name == "move_above":
                    target = step.get("object") or step.get("zone")
                    status = self.skills.move_above(target)
                elif skill_name == "move_to_zone":
                    status = self.skills.move_to_zone(step.get("zone"))
                elif skill_name == "open_gripper":
                    status = self.skills.open_gripper()
                elif skill_name == "close_gripper":
                    status = self.skills.close_gripper(step.get("object"))
                elif skill_name == "detect_objects":
                    res = self.skills.detect_objects()
                    self._log(f"    Phát hiện qua camera: {res}")
                    status = "SUCCESS"
                elif skill_name == "check_zone":
                    zone = step.get("zone", "")
                    occupant = self.skills.check_zone(zone)
                    if occupant:
                        self._log(f"    Vùng {zone}: Đang bị chiếm bởi '{occupant}'")
                    else:
                        self._log(f"    Vùng {zone}: Đang trống")
                    status = "SUCCESS"
                elif skill_name == "find_free_position":
                    pos_name = self.skills.find_free_position()
                    self._log(f"    Vị trí trống tìm thấy: '{pos_name}'")
                    status = "SUCCESS"
                else:
                    status = "INVALID_SKILL"
            except Exception as e:
                status = f"EXCEPTION ({e})"

            self._log(f"{call_repr} {dots} {status}")

            if status != "SUCCESS":
                all_success = False
                self._log(f"[ERROR] Dung thuc thi tai buoc: {call_repr} voi trang thai '{status}'")
                break

        if all_success:
            expected = {step["object"]: step["zone"] for step in plan
                        if step.get("skill") == "place"}
            if expected and not self.skills.verify_placements(expected):
                all_success = False
                self._log("[CAMERA] VERIFY_FAILED: cube chua o dung zone sau khi robot ve home")
        self._log("-" * 65)
        if all_success:
            self._log("TASK SUCCESS")
        else:
            self._log("TASK FAILED")
        self._log("=" * 65 + "\n")

        return all_success
