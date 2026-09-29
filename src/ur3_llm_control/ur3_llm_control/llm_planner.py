#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LLM Task Planner:
Ket noi toi 9Router API (tuong thich chuan OpenAI REST API v1) de chuyen doi
cau lenh ngon ngu tu nhien thanh Ke hoach co cau truc (JSON Structured Plan).
Co che do Offline Smart Fallback thong minh khong hardcode, tu dong tinh P = XX mod 6
cho bat ky ma sinh vien nao.
"""

import os
import re
import json
import yaml
import requests
import unicodedata
from typing import Dict, Any, Tuple, List

from .student_utils import parse_student_info, compute_optimal_sorting_plan


class LLMPlanner:
    """Module lap ke hoach nhiem vu su dung LLM qua 9Router hoac bo suy dien ngu nghia."""

    def __init__(self, config_dir: str = None):
        if not config_dir:
            try:
                from ament_index_python.packages import get_package_share_directory
                config_dir = os.path.join(get_package_share_directory("ur3_llm_control"), "config")
            except Exception:
                config_dir = os.path.join(
                    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config"
                )
        self.config_dir = config_dir
        self.llm_config = self._load_yaml("llm_config.yaml")
        self.student_config = self._load_yaml("student_config.yaml")
        self.scene_config = self._load_yaml("scene.yaml")

        self.base_url = self.llm_config.get("base_url", "https://api.9router.com/v1").rstrip("/")
        # Uu tien doc tu bien moi truong
        self.api_key = (
            os.environ.get("NINE_ROUTER_API_KEY")
            or os.environ.get("OPENAI_API_KEY")
            or self.llm_config.get("api_key", "").strip()
        )
        self.model = self.llm_config.get("model", "gpt-4o-mini")
        self.temperature = float(self.llm_config.get("temperature", 0.1))
        self.fallback_enabled = bool(self.llm_config.get("fallback_to_smart_planner", True))

        # Doc thong tin va tu dong tinh toan P = XX mod 6 cho bat ky sinh vien nao
        self.student_name = self.student_config.get("student_name", "Lê Anh Tuấn Bằng")
        self.student_id = str(self.student_config.get("student_id", "23020723"))
        self.xx, self.p_value, self.zone_mapping = parse_student_info(self.student_id)

        self.system_prompt = self._build_system_prompt()

    def _load_yaml(self, filename: str) -> Dict[str, Any]:
        filepath = os.path.join(self.config_dir, filename)
        if os.path.exists(filepath):
            with open(filepath, "r", encoding="utf-8") as f:
                return yaml.safe_load(f) or {}
        return {}

    def _build_system_prompt(self, scene_state: dict = None) -> str:
        cube_locs = {}
        zone_occupants = {}
        if scene_state:
            cube_locs = scene_state.get("cube_locations", {})
            zone_occupants = scene_state.get("zone_occupants", {})

        state_lines = []
        for obj in ["red_cube", "yellow_cube", "blue_cube"]:
            loc = cube_locs.get(obj, "source_tray")
            state_lines.append(f"  * {obj}: currently at '{loc}'")
        occ_str = json.dumps(zone_occupants) if zone_occupants else "all zones empty"

        prompt = f"""You are an advanced, optimal Task Planner for a 6-DOF Universal Robots UR3 manipulator.
Your job is to translate Natural Language Commands from the user into a STRICT JSON Structured Plan.

### HARD CONSTRAINTS:
1. You MUST ONLY use the allowed Robot Skills listed below.
2. DO NOT generate joint angles, trajectory coordinates, or low-level motor commands.
3. OUTPUT STRICT JSON ONLY with keys "thought" and "plan". No markdown formatting outside JSON.

### STUDENT & PERSONALIZATION CONTEXT:
- Student Name: {self.student_name}
- Student ID (MSSV): {self.student_id}
- Rule: XX = last two digits of Student ID = {self.xx}. P = {self.xx} mod 6 = {self.p_value}.
- Mapped Target Zones for P = {self.p_value}:
    * Zone A (zone_a) must receive: {self.zone_mapping['zone_a']}
    * Zone B (zone_b) must receive: {self.zone_mapping['zone_b']}
    * Zone C (zone_c) must receive: {self.zone_mapping['zone_c']}

### REAL-TIME WORKSPACE SCENE STATE:
{chr(10).join(state_lines)}
- Zone occupants: {occ_str}

### ALLOWED ROBOT SKILLS:
- pick: Pick an object from table. Args: "object" (string: red_cube, yellow_cube, blue_cube)
- place: Place the currently held object into target zone. Args: "object" (string), "zone" (string: zone_a, zone_b, zone_c, zone_temp)
- home: Move arm to home / observation pose. Args: none
- swap: Swap positions of two cubes using zone_temp. Args: "object_a", "object_b"
- stack: Stack object_top on top of object_bottom. Args: "object_top", "object_bottom"
- reset_scene: Reset all 3 cubes back to initial source trays. Args: none
- inspect_scene: Query status of all cubes and zones. Args: none
- clear_zone: Clear a specific zone. Args: "zone" (string)

### ALLOWED OBJECTS:
- "red_cube", "yellow_cube", "blue_cube"

### ALLOWED ZONES:
- "zone_a", "zone_b", "zone_c", "zone_temp"

### OPTIMAL PLANNING STRATEGY (CRITICAL FOR MINIMUM TIME & EXECUTION):
1. PRESERVE CORRECT POSITIONS: If an object is ALREADY in its designated target zone, DO NOT touch or move it!
2. NEVER return all cubes to waiting trays (DO NOT call clear_zones unless explicitly asked to reset).
3. DIRECT PLACEMENT: If an object is not in its target zone and its target zone is currently EMPTY, pick it and place it DIRECTLY into that target zone.
4. TWO-OBJECT CONFLICT / SWAP RESOLUTION:
   If two objects are in each other's target zones (e.g. obj1 is in target of obj2, and obj2 is in target of obj1):
   - Step 1: Pick obj1 and place it into "zone_temp".
   - Step 2: Pick obj2 and place it DIRECTLY into its correct target zone (do NOT put it into waiting tray).
   - Step 3: Pick obj1 from "zone_temp" and place it DIRECTLY into its correct target zone.
5. If user asks to move an object into a zone where it is already located, return {{"thought": "Object already in target zone.", "plan": [{{"skill": "home"}}]}}.
6. End all operational plans with {{"skill": "home"}}.
7. REJECTION: If user asks for an object or zone not in allowed lists (e.g., "quả táo", "vùng D"), return "plan": [] and explain why in "thought".

### OUTPUT JSON FORMAT:
{{
  "thought": "Reasoning explaining user intent, current state, and optimal steps in Vietnamese...",
  "plan": [
    {{"skill": "pick", "object": "red_cube"}},
    {{"skill": "place", "object": "red_cube", "zone": "zone_b"}},
    {{"skill": "home"}}
  ]
}}
"""
        return prompt

    def plan(self, user_command: str, scene_state: dict = None) -> Tuple[Dict[str, Any], str, str]:
        """
        Goi LLM (9Router) de lap ke hoach, co fallback tu dong.
        Bao cao ro rang che do Online API hay Offline.
        """
        user_command_clean = user_command.strip()
        last_error = None

        # 1. Thu goi 9Router API neu co api_key
        if self.api_key:
            # Thu ca URL tu config va localhost:20128/v1 (mac dinh cua local 9Router)
            candidate_urls = [self.base_url]
            if "localhost" not in self.base_url and "127.0.0.1" not in self.base_url:
                candidate_urls.append("http://localhost:20128/v1")

            for test_url in candidate_urls:
                try:
                    plan_dict = self._call_9router_api(user_command_clean, target_url=test_url, scene_state=scene_state)
                    if plan_dict and "plan" in plan_dict:
                        banner = f"ONLINE LLM (9Router @ {test_url} - Model: {self.model})"
                        conn_status = f"📡 [KẾT NỐI API THÀNH CÔNG] Đang lập kế hoạch qua 9Router Online (URL: {test_url} | Model: {self.model})"
                        print(f"\n[PLANNER MODE] >>> {banner} <<<", flush=True)
                        return plan_dict, banner, conn_status
                except Exception as e:
                    last_error = e

        # 2. Che do Offline Smart Planner (Fallback)
        if self.fallback_enabled:
            reason = f"Không thể kết nối tới 9Router API / Internet ({last_error})" if last_error else "Chưa cấu hình API Key 9Router"
            banner = "OFFLINE Smart Planner (Chế độ mô phỏng nội bộ)"
            conn_status = (
                f"⚠️ [CẢNH BÁO MẤT KẾT NỐI API]: {reason}.\n"
                f"   -> Hệ thống đang tự động sử dụng bộ lập kế hoạch nội bộ (Offline Smart Planner)!"
            )
            print(f"\n[PLANNER MODE] >>> {banner} <<<", flush=True)
            print(f"{conn_status}\n", flush=True)
            plan_dict = self._smart_rule_planner(user_command_clean, scene_state=scene_state)
            return plan_dict, banner, conn_status

        err_msg = (
            f"❌ [LỖI KẾT NỐI 9ROUTER]: Không thể kết nối tới 9Router Gateway tại '{self.base_url}' ({last_error}).\n"
            f"   -> Vui lòng mở một Terminal mới và chạy: 'npx 9router' để khởi động 9Router Local Gateway!"
        )
        print(f"\n{err_msg}\n", flush=True)
        return {"plan": []}, "9Router Connection Error", err_msg

    def _call_9router_api(self, user_command: str, target_url: str = None, scene_state: dict = None) -> Dict[str, Any]:
        """Gui HTTP Request chuan OpenAI Chat Completion toi 9Router."""
        endpoint = target_url or self.base_url
        url = f"{endpoint.rstrip('/')}/chat/completions"

        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.api_key}"
        }

        # Build dynamic prompt with real-time scene state
        system_prompt = self._build_system_prompt(scene_state=scene_state)

        payload = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_command}
            ],
            "temperature": self.temperature,
            "max_tokens": 1500,
            "stream": False
        }

        response = requests.post(url, headers=headers, json=payload, timeout=30)
        if response.status_code != 200:
            try:
                err_json = response.json()
                err_msg = err_json.get("error", {}).get("message", response.text)
            except Exception:
                err_msg = response.text
            raise RuntimeError(f"9Router trả về lỗi (Mã {response.status_code}): {err_msg}")

        data = response.json()
        content = data["choices"][0]["message"]["content"]
        return self._extract_json(content)

    def _extract_json(self, raw_text: str) -> Dict[str, Any]:
        """Trich xuat va chuan hoa JSON an toan tu phan hoi cua LLM."""
        raw_text = raw_text.strip()
        match = re.search(r"```(?:json)?\s*([\s\S]*?)\s*```", raw_text)
        if match:
            raw_text = match.group(1).strip()

        try:
            parsed = json.loads(raw_text)
        except json.JSONDecodeError:
            start = raw_text.find("{")
            end = raw_text.rfind("}")
            if start != -1 and end != -1:
                parsed = json.loads(raw_text[start : end + 1])
            else:
                raise

        # Chuan hoa cac buoc trong plan de tuong thich 100% voi TaskValidator
        if isinstance(parsed, dict) and "plan" in parsed and isinstance(parsed["plan"], list):
            normalized_plan = []
            for item in parsed["plan"]:
                if isinstance(item, dict):
                    skill_str = item.get("skill", "").strip()

                    # Xu ly truong hop LLM tra ve skill pick_and_place
                    if skill_str == "pick_and_place":
                        params = item.get("parameters", item)
                        obj = params.get("object", "")
                        tgt_z = params.get("end_zone", params.get("zone", params.get("target_zone", "")))
                        if obj and tgt_z:
                            normalized_plan.append({"skill": "pick", "object": obj})
                            normalized_plan.append({"skill": "place", "object": obj, "zone": tgt_z})
                            continue

                    # Neu LLM tra ve dang function call: pick(red_cube) hoac place(red_cube, zone_b)
                    func_match = re.match(r"^(\w+)\((.*)\)$", skill_str)
                    if func_match:
                        func_name = func_match.group(1)
                        args = [a.strip().strip("'\"") for a in func_match.group(2).split(",") if a.strip()]
                        new_item = {"skill": func_name}
                        if func_name == "pick" and len(args) >= 1:
                            new_item["object"] = args[0]
                        elif func_name == "place" and len(args) >= 2:
                            new_item["object"] = args[0]
                            new_item["zone"] = args[1]
                        elif func_name == "swap" and len(args) >= 2:
                            new_item["object_a"] = args[0]
                            new_item["object_b"] = args[1]
                        elif func_name == "stack" and len(args) >= 2:
                            new_item["object_top"] = args[0]
                            new_item["object_bottom"] = args[1]
                        elif func_name in ["move_above", "clear_zone"] and len(args) >= 1:
                            if func_name == "move_above":
                                new_item["object"] = args[0]
                            else:
                                new_item["zone"] = args[0]
                        elif func_name == "move_to_zone" and len(args) >= 1:
                            new_item["zone"] = args[0]
                        normalized_plan.append(new_item)
                    else:
                        normalized_plan.append(item)
                elif isinstance(item, str):
                    func_match = re.match(r"^(\w+)\((.*)\)$", item.strip())
                    if func_match:
                        func_name = func_match.group(1)
                        args = [a.strip().strip("'\"") for a in func_match.group(2).split(",") if a.strip()]
                        new_item = {"skill": func_name}
                        if func_name == "pick" and len(args) >= 1:
                            new_item["object"] = args[0]
                        elif func_name == "place" and len(args) >= 2:
                            new_item["object"] = args[0]
                            new_item["zone"] = args[1]
                        elif func_name == "swap" and len(args) >= 2:
                            new_item["object_a"] = args[0]
                            new_item["object_b"] = args[1]
                        elif func_name == "stack" and len(args) >= 2:
                            new_item["object_top"] = args[0]
                            new_item["object_bottom"] = args[1]
                        normalized_plan.append(new_item)
                    else:
                        normalized_plan.append({"skill": item.strip()})
            parsed["plan"] = normalized_plan

        return parsed

    def _smart_rule_planner(self, command: str, scene_state: dict = None) -> Dict[str, Any]:
        """
        Bo lap ke hoach noi bo dua tren phan tich ngu nghia (NLP/Semantic Extraction).
        KHONG HARDCODE: Tu dong trich xuat thuc the cho moi cau lenh tieng Viet / tieng Anh,
        dong thoi tinh toan P = XX mod 6 va ap dung thuat toan sap xep toi uu so buoc nhat.
        """
        cmd_raw = command.strip().lower()
        nfkd = unicodedata.normalize('NFKD', cmd_raw)
        cmd_clean = "".join([c for c in nfkd if not unicodedata.combining(c)]).replace('đ', 'd').replace('Đ', 'D')

        # 1. Kiem tra cau lenh Reset ban lam viec
        if any(kw in cmd_clean for kw in ["reset", "dat lai", "tra ve vi tri cu", "ve khay ban dau", "ve vi tri ban dau"]):
            thought = "Nguoi dung yeu cau reset toan bo cac khoi hop ve vi tri khay ban dau."
            return {"thought": thought, "plan": [{"skill": "reset_scene"}]}

        # 2. Kiem tra cau lenh Tra cuu trang thai (Inspect)
        if any(kw in cmd_clean for kw in ["inspect", "trang thai", "kiem tra vi tri", "bao cao", "xem vi tri"]):
            thought = "Nguoi dung yeu cau kiem tra vi tri cac khoi hop va zone tren ban."
            return {"thought": thought, "plan": [{"skill": "inspect_scene"}]}

        # 3. Kiem tra cau lenh Home
        if any(kw in cmd_clean for kw in ["home", "ve vi tri cho", "ve home", "ve nha", "dung cho"]):
            thought = "Nguoi dung yeu cau robot dua tay ve tu the cho (home)."
            return {"thought": thought, "plan": [{"skill": "home"}]}

        # 4. Kiem tra cau lenh Ca nhan hoa theo MSSV (DONG HOAN TOAN THEO MA SINH VIEN BAT KY & TOI UU TOI DA)
        if any(kw in cmd_clean for kw in [
            "student id", "mssv", "ma sinh vien", "ma so sinh vien", 
            "arrange all", "sap xep toan bo", "sap xep tat ca", "sap xep cac khoi", "theo ma"
        ]):
            cube_locs = {}
            if scene_state:
                cube_locs = scene_state.get("cube_locations", {})
            if not cube_locs:
                cube_locs = {"red_cube": "source_tray", "yellow_cube": "source_tray", "blue_cube": "source_tray"}

            plan_steps, thought_opt = compute_optimal_sorting_plan(cube_locs, self.zone_mapping)
            thought = (
                f"Sắp xếp theo MSSV {self.student_id} (XX={self.xx} -> P={self.p_value}). "
                f"Mục tiêu: Zone A -> {self.zone_mapping['zone_a']}, "
                f"Zone B -> {self.zone_mapping['zone_b']}, "
                f"Zone C -> {self.zone_mapping['zone_c']}. {thought_opt}"
            )
            return {"thought": thought, "plan": plan_steps}

        # Helper: Trich xuat cac khoi hop theo thu tu xuat hien trong cau lenh
        def extract_cubes_ordered(text: str) -> List[str]:
            patterns = {
                "red_cube": [
                    r"\bred_cube\b", r"\bkhoi mau do\b", r"\bmau do\b", r"\bkhoi do\b", r"\bred\b", r"\bdo\b"
                ],
                "yellow_cube": [
                    r"\byellow_cube\b", r"\bkhoi mau vang\b", r"\bmau vang\b", r"\bkhoi vang\b", r"\byellow\b", r"\bvang\b"
                ],
                "blue_cube": [
                    r"\bblue_cube\b", r"\bkhoi mau xanh lam\b", r"\bkhoi mau xanh duong\b", r"\bmau xanh lam\b",
                    r"\bmau xanh duong\b", r"\bxanh lam\b", r"\bxanh duong\b", r"\bkhoi xanh\b", r"\bblue\b", r"\bxanh\b"
                ]
            }
            matches = []
            for cube, pat_list in patterns.items():
                min_pos = 100000
                for pat in pat_list:
                    m = re.search(pat, text)
                    if m and m.start() < min_pos:
                        min_pos = m.start()
                if min_pos < 100000:
                    matches.append((min_pos, cube))
            matches.sort(key=lambda x: x[0])
            return [cube for _, cube in matches]

        cubes_in_cmd = extract_cubes_ordered(cmd_clean)

        # 5. Kiem tra cau lenh Doi cho (Swap)
        if any(kw in cmd_clean for kw in ["swap", "doi cho", "hoan doi", "trao doi", "switch"]):
            if len(cubes_in_cmd) >= 2:
                thought = f"Nguoi dung yeu cau hoan doi vi tri giua '{cubes_in_cmd[0]}' va '{cubes_in_cmd[1]}'."
                return {
                    "thought": thought,
                    "plan": [
                        {"skill": "swap", "object_a": cubes_in_cmd[0], "object_b": cubes_in_cmd[1]},
                        {"skill": "home"}
                    ]
                }

        # 6. Kiem tra cau lenh Xep chong (Stack)
        if any(kw in cmd_clean for kw in ["stack", "xep chong", "chong len", "len tren", "xep len", "dat len tren", "on top of"]):
            if len(cubes_in_cmd) >= 2:
                thought = f"Nguoi dung yeu cau xep chong khoi '{cubes_in_cmd[0]}' len tren '{cubes_in_cmd[1]}'."
                return {
                    "thought": thought,
                    "plan": [
                        {"skill": "stack", "object_top": cubes_in_cmd[0], "object_bottom": cubes_in_cmd[1]},
                        {"skill": "home"}
                    ]
                }

        # 7. Trich xuat Zone (A, B, C, Temp)
        target_zone = None
        zone_patterns = {
            "zone_a": [r"\bzone[_ ]?a\b", r"\bvung[_ ]?a\b", r"\bo[_ ]?a\b", r"\bkhu[_ ]?a\b", r"\bkhay[_ ]?a\b"],
            "zone_b": [r"\bzone[_ ]?b\b", r"\bvung[_ ]?b\b", r"\bo[_ ]?b\b", r"\bkhu[_ ]?b\b", r"\bkhay[_ ]?b\b"],
            "zone_c": [r"\bzone[_ ]?c\b", r"\bvung[_ ]?c\b", r"\bo[_ ]?c\b", r"\bkhu[_ ]?c\b", r"\bkhay[_ ]?c\b"],
            "zone_temp": [r"\bzone[_ ]?temp\b", r"\bvung[_ ]?tam\b", r"\bo[_ ]?tam\b", r"\bkhu[_ ]?tam\b", r"\bvung[_ ]?dem\b"]
        }
        for z_name, z_pats in zone_patterns.items():
            if any(re.search(pat, cmd_clean) for pat in z_pats):
                target_zone = z_name
                break

        # Kiem tra vat the khong hop le de validator phat hien
        invalid_obj = None
        for inv in ["qua tao", "apple", "khoi xanh la", "green", "green_cube", "trai tao", "qua bong", "ball"]:
            if inv in cmd_clean:
                invalid_obj = inv
                break

        # Kiem tra zone khong hop le de validator phat hien
        invalid_zone = None
        for inv_z in ["vung d", "zone d", "zone_d", "o d", "khu d", "vung e", "zone e", "zone_e", "o e", "khu e"]:
            if inv_z in cmd_clean:
                invalid_zone = inv_z.replace(" ", "_")
                break

        target_obj = cubes_in_cmd[0] if cubes_in_cmd else invalid_obj
        if not target_zone and invalid_zone:
            target_zone = invalid_zone

        if target_obj and target_zone:
            cube_locs = scene_state.get("cube_locations", {}) if scene_state else {}
            zone_occs = scene_state.get("zone_occupants", {}) if scene_state else {}

            cur_obj_loc = cube_locs.get(target_obj)
            cur_zone_occ = zone_occs.get(target_zone)

            # Neu vat da o dung zone yeu cau -> Khong thao tac gi them
            if cur_obj_loc == target_zone:
                thought = f"Vật '{target_obj}' đã ở sẵn trong '{target_zone}', robot giữ nguyên tư thế nghỉ (Home)."
                return {"thought": thought, "plan": [{"skill": "home"}]}

            plan_steps = []
            # Neu zone dich dang co vat the khac: tam thoi dua vat the do ra zone_temp
            if cur_zone_occ and cur_zone_occ != target_obj:
                plan_steps.append({"skill": "pick", "object": cur_zone_occ})
                plan_steps.append({"skill": "place", "object": cur_zone_occ, "zone": "zone_temp"})

            plan_steps.append({"skill": "pick", "object": target_obj})
            plan_steps.append({"skill": "place", "object": target_obj, "zone": target_zone})
            plan_steps.append({"skill": "home"})

            thought = f"Tối ưu quy trình: đưa '{target_obj}' vào '{target_zone}' không gây chồng đè."
            return {"thought": thought, "plan": plan_steps}
        elif target_obj and not target_zone:
            thought = f"Nguoi dung chi yeu cau gap vat '{target_obj}'."
            plan_steps = [
                {"skill": "pick", "object": target_obj},
                {"skill": "home"}
            ]
        else:
            thought = f"Khong the trich xuat hanh dong ro rang tu cau lenh: '{command}'."
            plan_steps = []
        return {"thought": thought, "plan": plan_steps}


