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
        for obj in ["red_cube", "yellow_cube", "blue_cube", "green_cube", "purple_cube"]:
            loc = cube_locs.get(obj, "source_tray")
            state_lines.append(f"  * {obj}: currently at '{loc}'")
        occ_str = json.dumps(zone_occupants) if zone_occupants else "all zones empty"

        prompt = f"""You are an advanced, optimal Task Planner for a 6-DOF Universal Robots UR3 manipulator equipped with an active 2-finger gripper and an overhead RGB Camera perception system.
Your job is to translate Natural Language Commands from the user into a STRICT JSON Structured Plan.

### HARD CONSTRAINTS:
1. You MUST ONLY use the allowed Robot Skills listed below.
2. DO NOT generate joint angles, trajectory coordinates, or low-level motor commands.
3. OUTPUT STRICT JSON ONLY with keys "thought" and "plan". No markdown formatting outside JSON.

### ALLOWED ROBOT SKILLS:
- detect_objects: Trigger camera to recognize positions of all 5 cubes. Args: none
- check_zone: Inspect if a zone is occupied. Args: "zone" (string: zone_a, zone_b, zone_c)
- find_free_position: Find an empty temporary buffer position. Args: none
- pick: Close physical gripper and attach cube. Args: "object" (string: red_cube, yellow_cube, blue_cube, green_cube, purple_cube)
- place: Open physical gripper and release cube into zone. Args: "object" (string), "zone" (string: zone_a, zone_b, zone_c, zone_temp_1, zone_temp_2, zone_temp_3, zone_temp)
- home: Move arm to safe observation / home pose. Args: none
- swap: Swap positions of two cubes using a temporary buffer. Args: "object_a", "object_b"
- stack: Stack object_top on top of object_bottom. Args: "object_top", "object_bottom"
- reset_scene: Reset all 5 cubes back to initial source trays. Args: none
- inspect_scene: Query status of all cubes and zones. Args: none
- clear_zone: Clear a specific zone. Args: "zone" (string)

### ALLOWED OBJECTS (5 BLOCKS):
- "red_cube", "yellow_cube", "blue_cube", "green_cube", "purple_cube"

### ALLOWED ZONES (3 TARGET ZONES + TEMPORARY BUFFERS):
- Target Zones: "zone_a", "zone_b", "zone_c"
- Temporary Zones: "zone_temp_1", "zone_temp_2", "zone_temp_3", "zone_temp"

### REAL-TIME WORKSPACE SCENE STATE (CAMERA & SENSORS):
{chr(10).join(state_lines)}
- Zone occupants: {occ_str}

### ⚠️ CRITICAL EXECUTION PRINCIPLES (CONFLICT RESOLUTION & CONSTRAINTS):
1. CAMERA-DRIVEN DIRECT PLANNING (KHÔNG DI CHUYỂN TAY MÁY ĐỂ KIỂM TRA THỪA):
   - Camera trên cao liên tục thu thập trạng thái thị giác thời gian thực.
   - Nếu ô đích TRỐNG (FREE): Robot lập tức thực hiện gắp và đặt trực tiếp:
     pick(target_cube) -> place(target_cube, target_zone) -> home().
     TUYỆT ĐỐI KHÔNG phát sinh bước di chuyển tay máy để kiểm tra nếu camera đã xác nhận ô trống!

2. AUTOMATIC CONFLICT RESOLUTION (CHỈ GIẢI PHÓNG KHI Ô ĐÍCH THỰC SỰ BỊ CHIẾM CHỖ):
   - Khi người dùng yêu cầu đưa một khối vào ô đích (ví dụ "Put red_cube in zone_c"):
     Nếu camera xác nhận ô đích đang bị khối khác chiếm giữ (ví dụ zone_c đang có blue_cube):
     * Robot giải phóng ô đích bằng cách dời vật cản sang vùng đệm tạm ("zone_temp_1", "zone_temp_2", hoặc "zone_temp_3"):
       1. pick("blue_cube") -> place("blue_cube", "zone_temp_1")
       2. pick("red_cube") -> place("red_cube", "zone_c")
       3. home()

3. USER EXPLICIT COMMAND SUPREMACY:
   - BẮT BUỘC tuân theo 100% đích đến người dùng yêu cầu.

4. BỎ QUA THAO TÁC THỪA:
   - Nếu khối đã ở sẵn trong ô yêu cầu, KHÔNG gắp lên thả lại. Luôn kết thúc bằng {{"skill": "home"}}.

### OUTPUT JSON FORMAT (KHI Ô ĐÍCH TRỐNG):
{{
  "thought": "Camera xác nhận zone_c hoàn toàn trống. Robot thực hiện gắp red_cube từ khay nguồn và đặt thẳng vào zone_c.",
  "plan": [
    {{"skill": "pick", "object": "red_cube"}},
    {{"skill": "place", "object": "red_cube", "zone": "zone_c"}},
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
                        non_home = [s for s in plan_dict["plan"] if s.get("skill") != "home"]
                        if not non_home and "bỏ qua" not in plan_dict.get("thought", "").lower():
                            plan_dict["thought"] += " [Tối ưu: Tất cả các vật yêu cầu đã ở đúng vị trí mục tiêu, bỏ qua các bước gắp thả thừa.]"
                        banner = f"ONLINE LLM (9Router @ {test_url} - Model: {self.model})"
                        conn_status = f"📡 [KẾT NỐI API THÀNH CÔNG] Đang lập kế hoạch qua 9Router Online (URL: {test_url} | Model: {self.model})"
                        print(f"\n[PLANNER MODE] >>> {banner} <<<", flush=True)
                        return plan_dict, banner, conn_status
                except Exception as e:
                    last_error = e

        # 2. Che do Offline Smart Planner: CHI KHI NGUOI DUNG CHO PHEP (fallback_enabled == True)
        if self.fallback_enabled:
            reason = f"Không thể kết nối tới 9Router API / Internet ({last_error})" if last_error else "Chưa cấu hình API Key 9Router"
            banner = "OFFLINE Smart Planner (Chế độ mô phỏng nội bộ - Đã được người dùng cho phép)"
            conn_status = (
                f"⚠️ [CẢNH BÁO MẤT KẾT NỐI API]: {reason}.\n"
                f"   -> Hệ thống đang sử dụng bộ lập kế hoạch nội bộ vì fallback_to_smart_planner=True."
            )
            print(f"\n[PLANNER MODE] >>> {banner} <<<", flush=True)
            print(f"{conn_status}\n", flush=True)
            plan_dict = self._smart_rule_planner(user_command_clean, scene_state=scene_state)
            plan_dict["plan"] = self._prune_redundant_moves(plan_dict.get("plan", []), scene_state=scene_state)
            return plan_dict, banner, conn_status

        # 3. Khi fallback_enabled == False (Mac dinh): TU CHOI CHAY OFFLINE VA BAO LOI RO RANG!
        err_msg = (
            f"❌ [LỖI KẾT NỐI 9ROUTER API]: Không thể kết nối tới 9Router LLM Gateway tại '{self.base_url}' ({last_error}).\n"
            f"   -> Chế độ Offline đã bị TẮT theo yêu cầu người dùng (100% Online LLM qua 9Router).\n"
            f"   -> Vui lòng kiểm tra terminal chạy 'npx 9router' để đảm bảo gateway đang hoạt động!"
        )
        print(f"\n{err_msg}\n", flush=True)
        return {"thought": f"Lỗi kết nối 9Router: {last_error}", "plan": []}, "9Router Connection Error", err_msg

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

        # Danh sach model online tren 9Router trong truong hop 1 model bi rate limit (429/503)
        candidate_models = [self.model]
        for fallback_m in ["gemini/gemini-3.1-flash-lite-preview", "gemini/gemini-3.5-flash-lite", "gemini/gemini-3.6-flash"]:
            if fallback_m not in candidate_models:
                candidate_models.append(fallback_m)

        last_resp_err = None
        for current_model in candidate_models:
            payload = {
                "model": current_model,
                "messages": [
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_command}
                ],
                "temperature": self.temperature,
                "max_tokens": 1500,
                "stream": False
            }

            try:
                response = requests.post(url, headers=headers, json=payload, timeout=25)
                if response.status_code == 200:
                    try:
                        data = response.json()
                        content = data["choices"][0]["message"]["content"]
                    except Exception:
                        # Fallback xu ly truong hop gateway tra ve Server-Sent Events (SSE chunk)
                        full_content = []
                        for line in response.text.splitlines():
                            line = line.strip()
                            if line.startswith("data:") and not line.endswith("[DONE]"):
                                chunk_str = line[5:].strip()
                                try:
                                    chunk = json.loads(chunk_str)
                                    delta = chunk.get("choices", [{}])[0].get("delta", {})
                                    if "content" in delta:
                                        full_content.append(delta["content"])
                                except Exception:
                                    pass
                        content = "".join(full_content)
                        if not content:
                            raise
                    self.model = current_model
                    return self._extract_json(content, scene_state=scene_state)
                elif response.status_code in [429, 503]:
                    try:
                        err_json = response.json()
                        err_msg = err_json.get("error", {}).get("message", response.text)
                    except Exception:
                        err_msg = response.text
                    last_resp_err = f"Model {current_model} bị quá tải ({response.status_code}): {err_msg}"
                    continue
                else:
                    try:
                        err_json = response.json()
                        err_msg = err_json.get("error", {}).get("message", response.text)
                    except Exception:
                        err_msg = response.text
                    raise RuntimeError(f"9Router trả về lỗi (Mã {response.status_code}): {err_msg}")
            except requests.exceptions.RequestException as req_err:
                raise req_err

        raise RuntimeError(last_resp_err or "Tất cả các model online trên 9Router đều không phản hồi.")

    def _extract_json(self, raw_text: str, scene_state: dict = None) -> Dict[str, Any]:
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
            last_picked = None
            for item in parsed["plan"]:
                if isinstance(item, dict):
                    skill_str = item.get("skill", "").strip()

                    # Xu ly truong hop LLM tra ve skill pick_and_place
                    if skill_str == "pick_and_place":
                        params = item.get("parameters", item)
                        obj = params.get("object", "")
                        tgt_z = params.get("end_pos", params.get("end_zone", params.get("zone", params.get("target_zone", ""))))
                        if obj and tgt_z:
                            normalized_plan.append({"skill": "pick", "object": obj})
                            normalized_plan.append({"skill": "place", "object": obj, "zone": tgt_z})
                            last_picked = obj
                            continue

                    # Neu LLM tra ve dang function call: pick(red_cube) hoac place(red_cube, zone_b) / place(zone_b)
                    func_match = re.match(r"^(\w+)\((.*)\)$", skill_str)
                    if func_match:
                        func_name = func_match.group(1)
                        args = [a.strip().strip("'\"") for a in func_match.group(2).split(",") if a.strip()]
                        new_item = {"skill": func_name}
                        if func_name == "pick" and len(args) >= 1:
                            new_item["object"] = args[0]
                            last_picked = args[0]
                        elif func_name == "place":
                            if len(args) >= 2:
                                new_item["object"] = args[0]
                                new_item["zone"] = args[1]
                                last_picked = args[0]
                            elif len(args) == 1:
                                new_item["object"] = last_picked
                                new_item["zone"] = args[0]
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
                        if item.get("skill") == "pick" and "object" in item:
                            last_picked = item["object"]
                        elif item.get("skill") == "place" and "object" not in item and last_picked:
                            item["object"] = last_picked
                        normalized_plan.append(item)
                elif isinstance(item, str):
                    func_match = re.match(r"^(\w+)\((.*)\)$", item.strip())
                    if func_match:
                        func_name = func_match.group(1)
                        args = [a.strip().strip("'\"") for a in func_match.group(2).split(",") if a.strip()]
                        new_item = {"skill": func_name}
                        if func_name == "pick" and len(args) >= 1:
                            new_item["object"] = args[0]
                            last_picked = args[0]
                        elif func_name == "place":
                            if len(args) >= 2:
                                new_item["object"] = args[0]
                                new_item["zone"] = args[1]
                                last_picked = args[0]
                            elif len(args) == 1:
                                new_item["object"] = last_picked
                                new_item["zone"] = args[0]
                        elif func_name == "swap" and len(args) >= 2:
                            new_item["object_a"] = args[0]
                            new_item["object_b"] = args[1]
                        elif func_name == "stack" and len(args) >= 2:
                            new_item["object_top"] = args[0]
                            new_item["object_bottom"] = args[1]
                        normalized_plan.append(new_item)
                    else:
                        normalized_plan.append({"skill": item.strip()})

            # Dam bao luon co buoc home cuoi cung de robot ve tu the an toan
            if normalized_plan and normalized_plan[-1].get("skill") != "home":
                normalized_plan.append({"skill": "home"})

            # Toi uu: Tu dong loai bo cac thao tac pick-and-place thua neu vat da o dung vi tri dich
            if scene_state:
                normalized_plan = self._prune_redundant_moves(normalized_plan, scene_state=scene_state)

            parsed["plan"] = normalized_plan

        return parsed

    def _prune_redundant_moves(self, plan: List[Dict[str, Any]], scene_state: dict = None) -> List[Dict[str, Any]]:
        """
        Loc bo cac buoc pick-and-place thua thai khi vat da o san vi tri dich:
        Neu vat the X da nam o zone Z, loai bo thao tac [pick(X), place(X, Z)].
        """
        if not plan or not scene_state:
            return plan

        cube_locs = dict(scene_state.get("cube_locations", {}))
        zone_occs = dict(scene_state.get("zone_occupants", {}))

        pruned = []
        skip_indices = set()
        n = len(plan)

        for i in range(n):
            if i in skip_indices:
                continue

            step = plan[i]
            skill = step.get("skill", "").strip().lower()

            if skill == "pick":
                obj = step.get("object")
                # Tim buoc place tiep theo cho vat obj nay
                place_idx = None
                for j in range(i + 1, n):
                    j_skill = plan[j].get("skill", "").strip().lower()
                    if j_skill == "place":
                        j_obj = plan[j].get("object") or obj
                        if j_obj == obj:
                            place_idx = j
                            break
                    elif j_skill == "pick":
                        break

                if place_idx is not None:
                    target_zone = plan[place_idx].get("zone")
                    cur_loc = cube_locs.get(obj)
                    is_in_zone = (cur_loc == target_zone) or (zone_occs.get(target_zone) == obj)

                    if is_in_zone and target_zone in ["zone_a", "zone_b", "zone_c", "zone_temp"]:
                        print(f"[OPTIMIZER] Bỏ qua thao tác thừa: '{obj}' đã ở sẵn '{target_zone}', không gắp lên rồi thả lại.", flush=True)
                        skip_indices.add(i)
                        skip_indices.add(place_idx)
                        for k in range(i + 1, place_idx):
                            k_skill = plan[k].get("skill", "").strip().lower()
                            if k_skill in ["move_above", "move_to_zone", "open_gripper", "close_gripper"]:
                                skip_indices.add(k)
                        continue

            if i not in skip_indices:
                pruned.append(step)
                if skill == "place":
                    p_obj = step.get("object")
                    p_zone = step.get("zone")
                    if p_obj and p_zone:
                        cube_locs[p_obj] = p_zone
                        zone_occs[p_zone] = p_obj

        non_home_steps = [s for s in pruned if s.get("skill") != "home"]
        if not non_home_steps:
            return [{"skill": "home"}]

        if pruned[-1].get("skill") != "home":
            pruned.append({"skill": "home"})

        return pruned

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
                    r"\bmau xanh duong\b", r"\bxanh lam\b", r"\bxanh duong\b", r"\bkhoi xanh\b", r"\bblue\b"
                ],
                "green_cube": [
                    r"\bgreen_cube\b", r"\bkhoi mau xanh la\b", r"\bkhoi mau luc\b", r"\bmau xanh la\b",
                    r"\bxanh la\b", r"\bxanh luc\b", r"\bgreen\b"
                ],
                "purple_cube": [
                    r"\bpurple_cube\b", r"\bkhoi mau tim\b", r"\bmau tim\b", r"\bkhoi tim\b", r"\bpurple\b", r"\btim\b"
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

        # 7. Trich xuat Zone (A, B, C, Temp 1, 2, 3)
        target_zone = None
        zone_patterns = {
            "zone_a": [r"\bzone[_ ]?a\b", r"\bvung[_ ]?a\b", r"\bo[_ ]?a\b", r"\bkhu[_ ]?a\b", r"\bkhay[_ ]?a\b"],
            "zone_b": [r"\bzone[_ ]?b\b", r"\bvung[_ ]?b\b", r"\bo[_ ]?b\b", r"\bkhu[_ ]?b\b", r"\bkhay[_ ]?b\b"],
            "zone_c": [r"\bzone[_ ]?c\b", r"\bvung[_ ]?c\b", r"\bo[_ ]?c\b", r"\bkhu[_ ]?c\b", r"\bkhay[_ ]?c\b"],
            "zone_temp_1": [r"\bzone[_ ]?temp[_ ]?1\b", r"\bo[_ ]?tam[_ ]?1\b", r"\bvung[_ ]?tam[_ ]?1\b"],
            "zone_temp_2": [r"\bzone[_ ]?temp[_ ]?2\b", r"\bo[_ ]?tam[_ ]?2\b", r"\bvung[_ ]?tam[_ ]?2\b"],
            "zone_temp_3": [r"\bzone[_ ]?temp[_ ]?3\b", r"\bo[_ ]?tam[_ ]?3\b", r"\bvung[_ ]?tam[_ ]?3\b"],
            "zone_temp": [r"\bzone[_ ]?temp\b", r"\bvung[_ ]?tam\b", r"\bo[_ ]?tam\b", r"\bkhu[_ ]?tam\b", r"\bvung[_ ]?dem\b"]
        }
        for z_name, z_pats in zone_patterns.items():
            if any(re.search(pat, cmd_clean) for pat in z_pats):
                target_zone = z_name
                break

        # Kiem tra vat the khong hop le de validator phat hien
        invalid_obj = None
        for inv in ["qua tao", "apple", "trai tao", "qua bong", "ball", "orange"]:
            if inv in cmd_clean:
                invalid_obj = inv
                break

        # Kiem tra zone khong hop le de validator phat hien
        invalid_zone = None
        for inv_z in ["vung d", "zone d", "zone_d", "o d", "khu d", "vung e", "zone e", "zone_e", "o e", "khu e"]:
            if inv_z in cmd_clean:
                invalid_zone = inv_z.replace(" ", "_")
                break

        # Kiem tra yeu cau dua vao o bat ky / ngau nhien
        if not target_zone and any(kw in cmd_clean for kw in ["bat ky", "o nao cung duoc", "vung bat ky", "any zone", "arbitrary", "ngau nhien"]):
            zone_occs = scene_state.get("zone_occupants", {}) if scene_state else {}
            for z in ["zone_a", "zone_b", "zone_c"]:
                if not zone_occs.get(z):
                    target_zone = z
                    break
            if not target_zone:
                target_zone = "zone_a"

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
                temp_dest = "zone_temp_1"
                for z_temp in ["zone_temp_1", "zone_temp_2", "zone_temp_3"]:
                    if not zone_occs.get(z_temp):
                        temp_dest = z_temp
                        break
                plan_steps.append({"skill": "pick", "object": cur_zone_occ})
                plan_steps.append({"skill": "place", "object": cur_zone_occ, "zone": temp_dest})
                thought = f"Camera phat hien '{target_zone}' dang bi '{cur_zone_occ}' chiem cho. Robot giai phong vat can sang '{temp_dest}', sau do gap '{target_obj}' vao '{target_zone}'."
            else:
                thought = f"Camera xac nhan '{target_zone}' hoan toan trong. Robot gap truc tiep '{target_obj}' va dat vao '{target_zone}' toi uu nhat."

            plan_steps.append({"skill": "pick", "object": target_obj})
            plan_steps.append({"skill": "place", "object": target_obj, "zone": target_zone})
            plan_steps.append({"skill": "home"})

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


