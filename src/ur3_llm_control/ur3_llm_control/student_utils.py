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
