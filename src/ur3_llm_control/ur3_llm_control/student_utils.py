#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Student Utils:
Tu dong tinh toan quy uoc ca nhan hoa tu bat ky Ma sinh vien (MSSV) nao:
P = XX mod 6 voi XX la 2 chu so cuoi cua MSSV.
"""

import re
from typing import Tuple, Dict


def parse_student_info(student_id: str) -> Tuple[int, int, Dict[str, str]]:
    """
    Tu dong trich xuat XX va tinh P = XX mod 6 tu bat ky MSSV nao.
    
    Tra ve:
        xx (int): Hai chu so cuoi MSSV
        p (int): P = xx % 6 (0 den 5)
        mapping (dict): Bang anh xa Zone -> Cube tuong ung
    """
    digits = re.findall(r'\d+', str(student_id))
    num_str = "".join(digits)
    xx = int(num_str[-2:]) if len(num_str) >= 2 else (int(num_str) if num_str else 0)
    p = xx % 6

    # Quy uoc de bai Lab 02:
    # P = 0 -> A = Red,    B = Yellow, C = Blue
    # P = 1 -> A = Red,    B = Blue,   C = Yellow
    # P = 2 -> A = Yellow, B = Red,    C = Blue
    # P = 3 -> A = Yellow, B = Blue,   C = Red
    # P = 4 -> A = Blue,   B = Red,    C = Yellow
    # P = 5 -> A = Blue,   B = Yellow, C = Red
    mappings = {
        0: {"zone_a": "red_cube", "zone_b": "yellow_cube", "zone_c": "blue_cube"},
        1: {"zone_a": "red_cube", "zone_b": "blue_cube", "zone_c": "yellow_cube"},
        2: {"zone_a": "yellow_cube", "zone_b": "red_cube", "zone_c": "blue_cube"},
        3: {"zone_a": "yellow_cube", "zone_b": "blue_cube", "zone_c": "red_cube"},
        4: {"zone_a": "blue_cube", "zone_b": "red_cube", "zone_c": "yellow_cube"},
        5: {"zone_a": "blue_cube", "zone_b": "yellow_cube", "zone_c": "red_cube"},
    }
    return xx, p, mappings[p]


def get_color_name(cube_name: str) -> str:
    """Tra ve ten mau tieng Viet cho cube."""
    if "red" in cube_name:
        return "Đỏ (Red)"
    elif "yellow" in cube_name:
        return "Vàng (Yellow)"
    elif "blue" in cube_name:
        return "Xanh lam (Blue)"
    return cube_name


def compute_optimal_sorting_plan(cube_locations: Dict[str, str], zone_mapping: Dict[str, str]) -> Tuple[list, str]:
    """
    Thuat toan lap ke hoach sap xep toi uu tuyet doi (Minimum Moves Optimization):
    1. Khoi nao da nam dung vi tri dich -> GIU NGUYEN (khong thao tac).
    2. Khoi chua dung vi tri ma zone dich dang trong -> gap THANG vao zone dich!
    3. Neu co xung dot hoan doi (e.g. 2 khoi doi cho nhau hoac chu trinh khep kin):
       - Gap 1 khoi tam vao zone_temp de giai phong 1 zone dich.
       - Khoi con lai di chuyen THANG vao vi tri dich vua duoc giai phong (khong dua ra cho nua).
       - Dua khoi tu zone_temp vao vi tri dich con lai.
    4. Toi uu hoa thoi gian va quy dao, khong can tra toan bo ve khay cho!
    """
    target_of_cube = {cube: zone for zone, cube in zone_mapping.items()}
    sim_locs = dict(cube_locations)
    sim_zones = {"zone_a": None, "zone_b": None, "zone_c": None, "zone_temp": None}
    for c, loc in sim_locs.items():
        if loc in sim_zones:
            sim_zones[loc] = c

    # Loc cac khoi da o dung vi tri
    already_correct = [c for c, tgt in target_of_cube.items() if sim_locs.get(c) == tgt]
    remaining = [c for c in target_of_cube if c not in already_correct]

    if not remaining:
        thought = "Tat ca cac khoi da o dung vi tri theo quy uoc MSSV, robot khong can di chuyen bat ky khoi nao."
        return [{"skill": "home"}], thought

    steps = []
    reason_notes = []
    if already_correct:
        reason_notes.append(f"Cac khoi da o dung vi tri giu nguyen: {already_correct}.")

    max_loops = 20
    loop_count = 0
    while remaining and loop_count < max_loops:
        loop_count += 1
        moved = False
        # 1. Uu tien di chuyen truc tiep neu target zone dang trong
        for c in list(remaining):
            tgt_z = target_of_cube[c]
            if sim_zones[tgt_z] is None:
                old_loc = sim_locs.get(c, "source_tray")
                steps.append({"skill": "pick", "object": c})
                steps.append({"skill": "place", "object": c, "zone": tgt_z})
                if old_loc in sim_zones:
                    sim_zones[old_loc] = None
                sim_zones[tgt_z] = c
                sim_locs[c] = tgt_z
                remaining.remove(c)
                reason_notes.append(f"Di chuyen truc tiep {c} tu {old_loc} vao {tgt_z}.")
                moved = True
                break

        # 2. Neu khong co zone nao trong -> co xung dot chu trinh (cycle), can giai phong 1 zone qua zone_temp
        if not moved:
            c_to_evict = None
            for c in remaining:
                cur_z = sim_locs.get(c)
                if cur_z in sim_zones and cur_z != "zone_temp":
                    c_to_evict = c
                    break
            if not c_to_evict:
                c_to_evict = remaining[0]

            old_loc = sim_locs.get(c_to_evict, "source_tray")
            steps.append({"skill": "pick", "object": c_to_evict})
            steps.append({"skill": "place", "object": c_to_evict, "zone": "zone_temp"})
            if old_loc in sim_zones:
                sim_zones[old_loc] = None
            sim_zones["zone_temp"] = c_to_evict
            sim_locs[c_to_evict] = "zone_temp"
            reason_notes.append(f"Giai phong {old_loc} bang cach tam dua {c_to_evict} ra zone_temp.")

    steps.append({"skill": "home"})
    thought = " ".join(reason_notes) + f" Hoan thanh ke hoach toi uu ({len(steps)} buoc)."
    return steps, thought

