"""Deterministic scene constraints applied after the LLM proposes skills."""

TEMP_ZONES = ("zone_temp_3", "zone_temp_2", "zone_temp_1")
TARGET_ZONES = ("zone_a", "zone_b", "zone_c")


def canonical_zone(zone):
    return "zone_temp_3" if zone == "zone_temp" else zone


def prepare_plan(plan, scene_state):
    """Insert a physical relocation before a pick whose destination is occupied.

    The LLM still chooses objects, destinations and order. This layer enforces the
    observed occupancy constraints; it never creates trajectories or joint goals.
    """
    occupants = {canonical_zone(z): obj for z, obj in
                 scene_state.get("zone_occupants", {}).items() if z != "zone_temp"}
    locations = {obj: canonical_zone(z) for obj, z in
                 scene_state.get("cube_locations", {}).items()}
    output = []
    held = scene_state.get("holding")
    for index, original in enumerate(plan):
        step = dict(original)
        if "zone" in step:
            step["zone"] = canonical_zone(step["zone"])
        skill = step.get("skill")
        if skill == "pick":
            obj = step.get("object")
            if held is not None:
                raise ValueError(f"Gripper dang giu {held}; khong the pick {obj}")
            target = None
            for later in plan[index + 1:]:
                if later.get("skill") == "pick":
                    break
                if later.get("skill") == "place" and later.get("object") == obj:
                    target = canonical_zone(later.get("zone"))
                    break
            blocker = occupants.get(target)
            if blocker and blocker != obj:
                buffer_zone = next((z for z in TEMP_ZONES + TARGET_ZONES if occupants.get(z) is None
                                    and z != target), None)
                if buffer_zone is None:
                    raise ValueError(f"Khong co vi tri tam trong de giai phong {target}")
                output.extend(({"skill": "pick", "object": blocker},
                               {"skill": "place", "object": blocker, "zone": buffer_zone}))
                old = locations.get(blocker)
                if old in occupants:
                    occupants[old] = None
                occupants[buffer_zone] = blocker
                locations[blocker] = buffer_zone
            old = locations.get(obj)
            if old in occupants and occupants[old] == obj:
                occupants[old] = None
            held = obj
        elif skill == "place":
            obj, zone = step.get("object"), step.get("zone")
            if held != obj:
                raise ValueError(f"Khong giu {obj} de dat vao {zone}")
            if occupants.get(zone) not in (None, obj):
                raise ValueError(f"{zone} van dang co {occupants[zone]}")
            occupants[zone] = obj
            locations[obj] = zone
            held = None
        output.append(step)
    return output


def order_independent_moves(plan, scene_state, zone_positions, user_command):
    """Choose the shortest order of independent pick/place pairs.

    Explicit sequencing words in the request are respected. Conflict resolution
    remains in prepare_plan and is intentionally not reordered here.
    """
    import itertools
    import math
    import re

    if re.search(r"\b(first|then|after|before|next|sequence|đầu tiên|sau đó|rồi|lần lượt)\b",
                 user_command.lower()):
        return plan
    work = [step for step in plan if step.get("skill") != "home"]
    if len(work) < 4 or len(work) % 2:
        return plan
    pairs = []
    for pick, place in zip(work[::2], work[1::2]):
        if (pick.get("skill") != "pick" or place.get("skill") != "place" or
                pick.get("object") != place.get("object")):
            return plan
        zone = canonical_zone(place.get("zone"))
        if scene_state.get("zone_occupants", {}).get(zone) not in (None, pick["object"]):
            return plan
        origin = scene_state.get("positions", {}).get(pick["object"])
        destination = zone_positions.get(zone)
        if not origin or not destination:
            return plan
        pairs.append((pick, place, origin, destination))
    if len({pair[0]["object"] for pair in pairs}) != len(pairs) or len(pairs) > 6:
        return plan

    def score(sequence):
        x, y = (0.24, 0.0)
        length = 0.0
        for _, _, source, target in sequence:
            length += math.hypot(source[0] - x, source[1] - y)
            length += math.hypot(target[0] - source[0], target[1] - source[1])
            x, y = target[:2]
        return length

    best = min(itertools.permutations(pairs), key=score)
    result = [step for pair in best for step in pair[:2]]
    if plan and plan[-1].get("skill") == "home":
        result.append(plan[-1])
    return result
