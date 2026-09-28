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

from .student_utils import parse_student_info


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

    def _build_system_prompt(self) -> str:
        prompt = f"""You are a high-level Task Planner for a 6-DOF Universal Robots UR3 manipulator.
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
- When user asks to arrange/sort objects according to Student ID (hoặc theo mã số sinh viên), generate the full sequence sorting all 3 cubes into their corresponding zones!

### ALLOWED ROBOT SKILLS:
- pick(object): Pick an object from table. Args: "object" (string)
- place(object, zone): Place the currently held object into target zone. Args: "object" (string), "zone" (string)
- home(): Move arm to home / observation pose. Args: none
- swap(object_a, object_b): Swap positions of two cubes using zone_temp. Args: "object_a", "object_b"
- stack(object_top, object_bottom): Stack object_top on top of object_bottom. Args: "object_top", "object_bottom"
- reset_scene(): Reset all 3 cubes back to initial source trays. Args: none
- inspect_scene(): Query status of all cubes and zones. Args: none
- move_above(object): Move gripper above object. Args: "object" (string)
- move_to_zone(zone): Move gripper above zone. Args: "zone" (string)
- open_gripper(): Open gripper fingers. Args: none
- close_gripper(): Close gripper fingers. Args: none

### ALLOWED OBJECTS:
- "red_cube"
- "yellow_cube"
- "blue_cube"

### ALLOWED ZONES:
- "zone_a"
- "zone_b"
- "zone_c"
- "zone_temp" (temporary holding spot for swaps/clearance)

### OUTPUT JSON FORMAT:
{{
  "thought": "Reasoning explaining user intent and steps...",
  "plan": [
    {{"skill": "pick", "object": "red_cube"}},
    {{"skill": "place", "object": "red_cube", "zone": "zone_b"}},
    {{"skill": "home"}}
  ]
}}
"""
        return prompt

    def plan(self, user_command: str) -> Tuple[Dict[str, Any], str]:
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
                    plan_dict = self._call_9router_api(user_command_clean, target_url=test_url)
                    if plan_dict and "plan" in plan_dict and len(plan_dict["plan"]) > 0:
                        banner = f"ONLINE LLM (9Router @ {test_url} - Model: {self.model})"
                        print(f"\n[PLANNER MODE] >>> {banner} <<<", flush=True)
                        return plan_dict, banner
                except Exception as e:
                    last_error = e

        # 2. Che do Offline Smart Planner (Fallback)
        if self.fallback_enabled:
            reason = f"Lý do: Không thể kết nối tới 9Router ({last_error}). Đang chuyển sang Smart Planner nội bộ." if last_error else "Chưa cấu hình API Key 9Router."
            banner = "OFFLINE Smart Planner (Chế độ mô phỏng độc lập)"
            print(f"\n[PLANNER MODE] >>> {banner} <<<", flush=True)
            print(f"[THÔNG BÁO] {reason}\n", flush=True)
            plan_dict = self._smart_rule_planner(user_command_clean)
            return plan_dict, banner

        return {"plan": []}, "No Planner Available"

    def _call_9router_api(self, user_command: str, target_url: str = None) -> Dict[str, Any]:
        """Gui HTTP Request chuan OpenAI Chat Completion toi 9Router."""
        endpoint = target_url or self.base_url
        url = f"{endpoint.rstrip('/')}/chat/completions"

        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.api_key}"
        }

        payload = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": user_command}
            ],
            "temperature": self.temperature,
            "max_tokens": 800
        }

        response = requests.post(url, headers=headers, json=payload, timeout=12)
        response.raise_for_status()

        data = response.json()
        content = data["choices"][0]["message"]["content"]
        return self._extract_json(content)

    def _extract_json(self, raw_text: str) -> Dict[str, Any]:
        """Trich xuat JSON an toan tu phan hoi cua LLM."""
        raw_text = raw_text.strip()
        match = re.search(r"```(?:json)?\s*([\s\S]*?)\s*```", raw_text)
        if match:
            raw_text = match.group(1).strip()

        try:
            return json.loads(raw_text)
        except json.JSONDecodeError:
            start = raw_text.find("{")
            end = raw_text.rfind("}")
            if start != -1 and end != -1:
                return json.loads(raw_text[start : end + 1])
            raise

    def _smart_rule_planner(self, command: str) -> Dict[str, Any]:
        """
        Bo lap ke hoach noi bo dua tren phan tich ngu nghia (NLP/Semantic Extraction).
        KHONG HARDCODE: Tu dong trich xuat thuc the (entity) cho moi cau lenh tieng Viet / tieng Anh,
        dong thoi tu dong tinh toan P = XX mod 6 cho bat ky MSSV nao.
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

        # 4. Kiem tra cau lenh Ca nhan hoa theo MSSV (DONG HOAN TOAN THEO MA SINH VIEN BAT KY)
        if any(kw in cmd_clean for kw in [
            "student id", "mssv", "ma sinh vien", "ma so sinh vien", 
            "arrange all", "sap xep toan bo", "sap xep tat ca", "sap xep cac khoi", "theo ma"
        ]):
            thought = (
                f"Cau lenh yeu cau sap xep theo MSSV {self.student_id} (XX={self.xx} -> P={self.p_value}): "
                f"Zone A -> {self.zone_mapping['zone_a']}, "
                f"Zone B -> {self.zone_mapping['zone_b']}, "
                f"Zone C -> {self.zone_mapping['zone_c']}."
            )
            plan_steps = [
                {"skill": "pick", "object": self.zone_mapping["zone_a"]},
                {"skill": "place", "object": self.zone_mapping["zone_a"], "zone": "zone_a"},
                {"skill": "pick", "object": self.zone_mapping["zone_b"]},
                {"skill": "place", "object": self.zone_mapping["zone_b"], "zone": "zone_b"},
                {"skill": "pick", "object": self.zone_mapping["zone_c"]},
                {"skill": "place", "object": self.zone_mapping["zone_c"], "zone": "zone_c"},
                {"skill": "home"}
            ]
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
            thought = f"Nguoi dung yeu cau gap vat '{target_obj}' dat vao vung '{target_zone}'."
            plan_steps = [
                {"skill": "pick", "object": target_obj},
                {"skill": "place", "object": target_obj, "zone": target_zone},
                {"skill": "home"}
            ]
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


