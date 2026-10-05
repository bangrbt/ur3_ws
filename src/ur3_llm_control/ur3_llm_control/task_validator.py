#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Task Validator: Kiem tra tinh hop le cua ke hoach do LLM sinh ra.
Chi cho phep cac skill, object, zone nam trong whitelist.
Phat hien va tu choi cac ke hoach vi pham logic hoac tham so khong hop le.
"""

from typing import Tuple, List, Dict, Any

ALLOWED_SKILLS = {
    "home",
    "pick",
    "place",
    "move_above",
    "open_gripper",
    "close_gripper",
    "move_to_zone",
    "swap",
    "stack",
    "reset_scene",
    "inspect_scene",
    "clear_zones",
    "clear_zone",
    "detect_objects",
    "check_zone",
    "find_free_position"
}

ALLOWED_OBJECTS = {
    "red_cube",
    "yellow_cube",
    "blue_cube",
    "green_cube",
    "purple_cube"
}

ALLOWED_ZONES = {
    "zone_a",
    "zone_b",
    "zone_c",
    "zone_temp",
    "zone_temp_1",
    "zone_temp_2",
    "zone_temp_3"
}


class TaskValidator:
    """Kiem tra va xac thuc tinh dung dan cua Plan truoc khi thuc thi."""

    def __init__(self):
        self.allowed_skills = ALLOWED_SKILLS
        self.allowed_objects = ALLOWED_OBJECTS
        self.allowed_zones = ALLOWED_ZONES

    def validate_plan(self, plan_data: Dict[str, Any], initial_holding: str = None) -> Tuple[bool, str]:
        """
        Xac thuc toan bo chuoi skill trong plan_data.
        
        Args:
            plan_data: Dict chua key 'plan' la danh sach cac action.
            initial_holding: Vat the ma tay kep dang giu ban dau (neu co).
            
        Returns:
            Tuple (is_valid: bool, reason: str)
        """
        if not isinstance(plan_data, dict):
            return False, "Plan data phai la mot JSON object (dictionary)."

        if "plan" not in plan_data or not isinstance(plan_data["plan"], list):
            return False, "Kế hoạch thieu key 'plan' hoac 'plan' khong phai la mot danh sach cac buoc."

        actions: List[Dict[str, Any]] = plan_data["plan"]
        if len(actions) == 0:
            thought = plan_data.get("thought", "").strip()
            if thought:
                return False, f"Yêu cầu bị từ chối: {thought}"
            return False, "Kế hoạch rỗng (không có bước nào cần thực thi)."

        # Theo doi trang thai ao de kiem tra tien de / hau de (Preconditions / Postconditions)
        simulated_holding = initial_holding

        for idx, step in enumerate(actions, 1):
            if not isinstance(step, dict):
                return False, f"Buoc {idx} khong hop le (phai la mot dictionary)."

            skill = step.get("skill", "").strip().lower()
            if not skill:
                return False, f"Buoc {idx} thieu ten 'skill'."

            # 1. Kiem tra Whitelist Skill
            if skill not in self.allowed_skills:
                return False, f"Buoc {idx}: Skill '{skill}' khong hop le! Chi chap nhan cac skill: {sorted(list(self.allowed_skills))}."

            # 2. Kiem tra tham so cua tung Skill
            if skill == "pick":
                obj = step.get("object", "")
                if not obj or obj not in self.allowed_objects:
                    return False, f"Buoc {idx}: Object '{obj}' khong hop le trong pick! Cac vat the hop le: {sorted(list(self.allowed_objects))}."
                if simulated_holding is not None:
                    return False, f"Buoc {idx}: Xung dot logic - Tay kep dang giu '{simulated_holding}', khong the pick them '{obj}'!"
                simulated_holding = obj

            elif skill == "place":
                obj = step.get("object", "")
                zone = step.get("zone", "")
                if not obj or obj not in self.allowed_objects:
                    return False, f"Buoc {idx}: Object '{obj}' khong hop le trong place! Cac vat the hop le: {sorted(list(self.allowed_objects))}."
                if not zone or zone not in self.allowed_zones:
                    return False, f"Buoc {idx}: Zone '{zone}' khong hop le trong place! Cac vung hop le: {sorted(list(self.allowed_zones))}."
                if simulated_holding != obj:
                    return False, f"Buoc {idx}: Xung dot logic - Robot hien khong giu '{obj}' (dang giu '{simulated_holding}'), khong the place!"
                simulated_holding = None

            elif skill == "swap":
                obj_a = step.get("object_a") or step.get("object1", "")
                obj_b = step.get("object_b") or step.get("object2", "")
                if obj_a not in self.allowed_objects or obj_b not in self.allowed_objects:
                    return False, f"Buoc {idx}: Swap can 2 vat the hop le (object_a, object_b)."
                if obj_a == obj_b:
                    return False, f"Buoc {idx}: Swap yeu cau 2 vat the khac nhau."

            elif skill == "stack":
                top = step.get("object_top") or step.get("top", "")
                bottom = step.get("object_bottom") or step.get("bottom", "")
                if top not in self.allowed_objects or bottom not in self.allowed_objects:
                    return False, f"Buoc {idx}: Stack can vat 'object_top' va 'object_bottom' hop le."
                if top == bottom:
                    return False, f"Buoc {idx}: Khong the xep chong 1 vat len chinh no."

            elif skill == "move_above":
                obj = step.get("object", "")
                zone = step.get("zone", "")
                if not obj and not zone:
                    return False, f"Buoc {idx}: move_above can tham so 'object' hoac 'zone'."
                if obj and obj not in self.allowed_objects:
                    return False, f"Buoc {idx}: Object '{obj}' khong hop le."
                if zone and zone not in self.allowed_zones:
                    return False, f"Buoc {idx}: Zone '{zone}' khong hop le."

            elif skill == "move_to_zone":
                zone = step.get("zone", "")
                if not zone or zone not in self.allowed_zones:
                    return False, f"Buoc {idx}: Zone '{zone}' khong hop le! Cac vung hop le: {sorted(list(self.allowed_zones))}."

            elif skill == "clear_zone":
                zone = step.get("zone", "")
                if zone and zone not in self.allowed_zones:
                    return False, f"Buoc {idx}: Zone '{zone}' khong hop le trong clear_zone! Cac vung hop le: {sorted(list(self.allowed_zones))}."

            elif skill == "clear_zones":
                # Clear all zones does not require parameters
                pass

            elif skill == "check_zone":
                zone = step.get("zone", "")
                if not zone or zone not in self.allowed_zones:
                    return False, f"Buoc {idx}: check_zone can tham so 'zone' hop le! Cac vung hop le: {sorted(list(self.allowed_zones))}."

            elif skill in {"detect_objects", "find_free_position"}:
                pass

        return True, "Kế hoạch hợp lệ 100% theo quy chuẩn."
