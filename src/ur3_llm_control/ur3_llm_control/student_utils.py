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
    """Sort three targets while accounting for all five cubes and buffer occupancy."""
    from .plan_safety import TEMP_ZONES, TARGET_ZONES, canonical_zone

    locations = {cube: canonical_zone(loc) for cube, loc in cube_locations.items()}
    occupants = {zone: None for zone in TARGET_ZONES + TEMP_ZONES}
    for cube, zone in locations.items():
        if zone in occupants:
            if occupants[zone] is not None:
                raise ValueError(f"Hai vat duoc bao o cung {zone}")
            occupants[zone] = cube

    steps = []
    def relocate(cube, destination):
        source = locations.get(cube)
        if occupants[destination] is not None:
            raise ValueError(f"{destination} dang bi chiem")
        steps.extend(({"skill": "pick", "object": cube},
                      {"skill": "place", "object": cube, "zone": destination}))
        if source in occupants:
            occupants[source] = None
        occupants[destination] = cube
        locations[cube] = destination

    for _ in range(12):
        pending = [(zone, cube) for zone, cube in zone_mapping.items()
                   if locations.get(cube) != zone]
        if not pending:
            steps.append({"skill": "home"})
            return steps, f"Sap xep xong 3 zone, co {len(steps) - 1} buoc thao tac."
        for zone, cube in pending:
            if occupants[zone] is None:
                relocate(cube, zone)
                break
        else:
            # A cycle, or a non-target cube occupies a target zone.
            zone, _ = pending[0]
            blocker = occupants[zone]
            buffer_zone = next((z for z in TEMP_ZONES if occupants[z] is None), None)
            if blocker is None or buffer_zone is None:
                raise ValueError("Khong con vi tri tam trong de sap xep")
            relocate(blocker, buffer_zone)
    raise ValueError("Khong the hoan thanh ke hoach sap xep")
