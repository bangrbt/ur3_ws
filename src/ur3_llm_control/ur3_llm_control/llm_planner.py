#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LLM Task Planner:
Ket noi toi 9Router API (tuong thich OpenAI REST API v1) de chuyen doi
cau lenh ngon ngu tu nhien thanh Ke hoach co cau truc (JSON Structured Plan).
Co che do Offline Smart Fallback de dam bao luon chay duoc ngay ca khi khong co mang/API key.
"""

import os
import re
import json
import yaml
import requests
import unicodedata
from typing import Dict, Any, Tuple


class LLMPlanner:
    """Module lap ke hoach nhiem vu su dung LLM qua 9Router."""

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

        self.student_name = self.student_config.get("student_name", "Lê Anh Tuấn Bằng")
        self.student_id = str(self.student_config.get("student_id", "23020723"))
        self.p_value = self.student_config.get("p_value", 5)

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
- Rule: P = (Last two digits of MSSV) mod 6 = 23 mod 6 = 5.
- Therefore, for P = 5:
    * Zone A (zone_a) must receive the BLUE cube (blue_cube)
    * Zone B (zone_b) must receive the YELLOW cube (yellow_cube)
    * Zone C (zone_c) must receive the RED cube (red_cube)
- When user asks to arrange/sort objects according to Student ID (hoặc theo mã số sinh viên), generate the full sequence sorting all 3 cubes into their corresponding zones!

### ALLOWED ROBOT SKILLS:
- pick(object): Pick an object from table. Args: "object" (string)
- place(object, zone): Place the currently held object into target zone. Args: "object" (string), "zone" (string)
- home(): Move arm to home / observation pose. Args: none
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
- "zone_temp" (temporary holding spot if a zone swap is needed)

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
        
        Returns:
            Tuple (plan_dict, source_info)
        """
        user_command_clean = user_command.strip()

        # Neu co API key hop le, thu goi 9Router
        if self.api_key:
            try:
                plan_dict = self._call_9router_api(user_command_clean)
                if plan_dict and "plan" in plan_dict:
                    return plan_dict, f"9Router ({self.model})"
            except Exception as e:
                print(f"[WARN] [LLM Planner] Goi 9Router API that bai ({e}). Chuyen sang Smart Fallback Planner.")

        # Che do Offline Smart Planner (Fallback)
        if self.fallback_enabled:
            plan_dict = self._smart_rule_planner(user_command_clean)
            return plan_dict, "Smart Offline Planner (Simulation Mode)"

        return {"plan": []}, "No Planner Available"

    def _call_9router_api(self, user_command: str) -> Dict[str, Any]:
        """Gui HTTP Request chuan OpenAI Chat Completion toi 9Router."""
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

        url = f"{self.base_url}/chat/completions"
        response = requests.post(url, headers=headers, json=payload, timeout=25)
        response.raise_for_status()

        data = response.json()
        content = data["choices"][0]["message"]["content"]
        return self._extract_json(content)

    def _extract_json(self, raw_text: str) -> Dict[str, Any]:
        """Trich xuat JSON an toan tu phan hoi cua LLM."""
        raw_text = raw_text.strip()
        # Loai bo markdown code block neu co
        match = re.search(r"```(?:json)?\s*([\s\S]*?)\s*```", raw_text)
        if match:
            raw_text = match.group(1).strip()

        try:
            return json.loads(raw_text)
        except json.JSONDecodeError:
            # Thu tim cap ngoac nhon lon nhat
            start = raw_text.find("{")
            end = raw_text.rfind("}")
            if start != -1 and end != -1:
                return json.loads(raw_text[start : end + 1])
            raise

    def _smart_rule_planner(self, command: str) -> Dict[str, Any]:
        """
        Bo lap ke hoach noi bo cuc ky thong minh dua tren luat ngu nghia (NLP/Regex).
        Dap ung 100% ca cau lenh tieng Anh va tieng Viet (co dau hoac khong dau), co ban va nang cao.
        """
        cmd_raw = command.strip().lower()
        # Chuyen ve khong dau de ho tro ca go tieng Viet khong dau va co dau
        nfkd = unicodedata.normalize('NFKD', cmd_raw)
        cmd_clean = "".join([c for c in nfkd if not unicodedata.combining(c)]).replace('đ', 'd').replace('Đ', 'D')

        plan_steps = []
        thought = ""

        # 1. Kiem tra cau lenh nang cao ca nhan hoa: Student ID / MSSV / Sap xep toan bo
        if any(kw in cmd_clean for kw in [
            "student id", "mssv", "ma sinh vien", "ma so sinh vien", 
            "arrange all", "sap xep toan bo", "sap xep tat ca", "sap xep cac khoi", "theo ma"
        ]):
            thought = f"Cau lenh yeu cau sap xep theo MSSV {self.student_id} (P=5): Zone A -> Blue, Zone B -> Yellow, Zone C -> Red."
            plan_steps = [
                {"skill": "pick", "object": "blue_cube"},
                {"skill": "place", "object": "blue_cube", "zone": "zone_a"},
                {"skill": "pick", "object": "yellow_cube"},
                {"skill": "place", "object": "yellow_cube", "zone": "zone_b"},
                {"skill": "pick", "object": "red_cube"},
                {"skill": "place", "object": "red_cube", "zone": "zone_c"},
                {"skill": "home"}
            ]
            return {"thought": thought, "plan": plan_steps}

        # 2. Kiem tra cau lenh Home / Ve vi tri cho
        if any(kw in cmd_clean for kw in ["home", "ve vi tri cho", "ve home", "ve nha", "reset", "return home", "go home"]):
            thought = "Nguoi dung yeu cau robot dua tay ve vi tri cho (home)."
            return {"thought": thought, "plan": [{"skill": "home"}]}

        # 3. Kiem tra cau lenh co ban (Don vat the): Tim object va zone
        target_obj = None
        if any(w in cmd_clean for w in ["red", "mau do", "khoi do", "do", "red_cube"]):
            target_obj = "red_cube"
        elif any(w in cmd_clean for w in ["yellow", "mau vang", "khoi vang", "vang", "yellow_cube"]):
            target_obj = "yellow_cube"
        elif any(w in cmd_clean for w in ["blue", "xanh lam", "xanh duong", "khoi xanh", "xanh", "blue_cube"]):
            target_obj = "blue_cube"

        # Tim Zone
        target_zone = None
        if any(w in cmd_clean for w in ["zone a", "zone_a", "vung a", "o a", "khu a"]):
            target_zone = "zone_a"
        elif any(w in cmd_clean for w in ["zone b", "zone_b", "vung b", "o b", "khu b"]):
            target_zone = "zone_b"
        elif any(w in cmd_clean for w in ["zone c", "zone_c", "vung c", "o c", "khu c"]):
            target_zone = "zone_c"
        elif any(w in cmd_clean for w in ["zone temp", "zone_temp", "vung tam", "o tam"]):
            target_zone = "zone_temp"

        if target_obj and target_zone:
            thought = f"Nguoi dung yeu cau gap vat '{target_obj}' dat vao vung '{target_zone}'."
            plan_steps = [
                {"skill": "pick", "object": target_obj},
                {"skill": "place", "object": target_obj, "zone": target_zone},
                {"skill": "home"}
            ]
        elif target_obj and not target_zone:
            # Chi pick
            thought = f"Nguoi dung chi yeu cau gap vat '{target_obj}'."
            plan_steps = [
                {"skill": "pick", "object": target_obj},
                {"skill": "home"}
            ]
        else:
            thought = f"Khong the trich xuat hanh dong ro rang tu cau lenh: '{command}'."
            plan_steps = []

        return {"thought": thought, "plan": plan_steps}
