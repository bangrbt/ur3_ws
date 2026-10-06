from itertools import permutations

import pytest

from ur3_llm_control.plan_safety import prepare_plan, order_independent_moves
from ur3_llm_control.student_utils import compute_optimal_sorting_plan
from ur3_llm_control.task_validator import TaskValidator


CUBES = ("red_cube", "yellow_cube", "blue_cube", "green_cube", "purple_cube")
MAPPING = {"zone_a": "blue_cube", "zone_b": "yellow_cube", "zone_c": "red_cube"}
ZONES = ("zone_a", "zone_b", "zone_c", "zone_temp_1", "zone_temp_2", "zone_temp_3")


def scene(locations):
    return {"cube_locations": locations, "zone_occupants":
            {z: next((c for c, position in locations.items() if position == z), None)
                for z in ZONES}}


def test_occupied_destination_is_evacuated_before_target_pick():
    state = scene({"red_cube": "source_tray", "blue_cube": "zone_b",
                   "yellow_cube": "source_tray", "green_cube": "source_tray",
                   "purple_cube": "source_tray"})
    proposed = [{"skill": "pick", "object": "red_cube"},
                {"skill": "place", "object": "red_cube", "zone": "zone_b"},
                {"skill": "home"}]
    repaired = prepare_plan(proposed, state)
    assert repaired[:2] == [{"skill": "pick", "object": "blue_cube"},
                             {"skill": "place", "object": "blue_cube", "zone": "zone_temp_3"}]
    assert TaskValidator().validate_plan({"plan": repaired}, scene_state=state)[0]
    assert not TaskValidator().validate_plan({"plan": proposed}, scene_state=state)[0]


def test_full_temporary_zones_use_a_free_target_zone():
    state = scene({"red_cube": "source_tray", "blue_cube": "zone_b",
                   "yellow_cube": "zone_temp_1", "green_cube": "zone_temp_2",
                   "purple_cube": "zone_temp_3"})
    plan = prepare_plan([{"skill": "pick", "object": "red_cube"},
                         {"skill": "place", "object": "red_cube", "zone": "zone_b"}], state)
    assert plan[:2] == [{"skill": "pick", "object": "blue_cube"},
                        {"skill": "place", "object": "blue_cube", "zone": "zone_a"}]
    assert TaskValidator().validate_plan({"plan": plan}, scene_state=state)[0]


def test_no_free_zone_fails_closed():
    state = scene({"red_cube": "source_tray", "blue_cube": "zone_b",
                   "yellow_cube": "zone_temp_1", "green_cube": "zone_temp_2",
                   "purple_cube": "zone_temp_3"})
    state["zone_occupants"]["zone_a"] = "unrecognized_cube"
    state["zone_occupants"]["zone_c"] = "unrecognized_cube"
    with pytest.raises(ValueError, match="Khong co vi tri tam"):
        prepare_plan([{"skill": "pick", "object": "red_cube"},
                      {"skill": "place", "object": "red_cube", "zone": "zone_b"}], state)


def test_student_sorting_all_five_cube_placements():
    # Every injective placement of five cubes across six physical zones.
    validator = TaskValidator()
    for arrangement in permutations(ZONES, len(CUBES)):
        locations = dict(zip(CUBES, arrangement))
        initial = scene(locations)
        plan, _ = compute_optimal_sorting_plan(locations, MAPPING)
        assert len(plan) <= 13
        valid, reason = validator.validate_plan({"plan": plan}, scene_state=initial)
        assert valid, (locations, reason)
        end = dict(locations)
        for index in range(0, len(plan) - 1, 2):
            if plan[index]["skill"] == "pick":
                end[plan[index]["object"]] = plan[index + 1]["zone"]
        assert all(end[cube] == zone for zone, cube in MAPPING.items())


def test_route_order_respects_explicit_sequence():
    plan = [{"skill": "pick", "object": "purple_cube"},
            {"skill": "place", "object": "purple_cube", "zone": "zone_a"},
            {"skill": "pick", "object": "yellow_cube"},
            {"skill": "place", "object": "yellow_cube", "zone": "zone_b"},
            {"skill": "home"}]
    state = scene({"purple_cube": "source_tray", "yellow_cube": "source_tray"})
    state["positions"] = {"purple_cube": [0.24, 0.22], "yellow_cube": [0.24, 0.0]}
    destinations = {"zone_a": [0.35, -0.11], "zone_b": [0.35, 0.0]}
    assert order_independent_moves(plan, state, destinations, "First purple, then yellow") == plan
    optimized = order_independent_moves(plan, state, destinations, "Arrange both cubes")
    assert optimized[0]["object"] == "yellow_cube"


def test_malformed_llm_step_fails_validation_without_exception():
    validator = TaskValidator()
    assert not validator.validate_plan({"plan": [{"skill": 42}]})[0]
    assert not validator.validate_plan({"plan": []}, scene_state=scene({}))[0]
