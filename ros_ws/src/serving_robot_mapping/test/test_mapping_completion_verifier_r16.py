from serving_robot_mapping.mapping_completion_verifier import (
    verify_closed_free_region,
)
from serving_robot_mapping.exploration_goal_manager import (
    ExplorationGoalManager,
)
from serving_robot_mapping.mapping_state_machine import (
    MappingState,
    MappingStateMachine,
)


def enclosed_map(size=10):
    cells = [100] * (size * size)
    for y in range(2, size - 2):
        for x in range(2, size - 2):
            cells[y * size + x] = 0
    return cells


def test_enclosed_free_space_passes():
    cells = enclosed_map()
    ok, reason = verify_closed_free_region(
        cells, 10, 10, 1.0, min_free_area_m2=10.0
    )
    assert ok
    assert reason == "CLOSED_FREE_REGION_VERIFIED"


def test_exposed_unknown_prevents_completion():
    cells = enclosed_map()
    cells[3 * 10 + 4] = -1
    ok, reason = verify_closed_free_region(
        cells, 10, 10, 1.0, min_free_area_m2=10.0
    )
    assert not ok
    assert reason == "FREE_SPACE_TOUCHES_UNKNOWN"


def test_tiny_map_prevents_completion():
    cells = enclosed_map()
    ok, reason = verify_closed_free_region(
        cells, 10, 10, 0.1, min_free_area_m2=10.0
    )
    assert not ok
    assert reason == "INSUFFICIENT_KNOWN_FREE_AREA"


def test_free_space_at_map_edge_prevents_completion():
    cells = enclosed_map()
    cells[4 * 10] = 0
    ok, reason = verify_closed_free_region(
        cells, 10, 10, 1.0, min_free_area_m2=10.0
    )
    assert not ok
    assert reason == "FREE_SPACE_REACHES_MAP_EDGE"


def test_completion_requires_independent_evidence():
    machine = MappingStateMachine()
    machine.transition_to(MappingState.EXPLORING)
    goals = ExplorationGoalManager(
        machine, no_frontier_cycles=2, stable_time_s=1.0
    )

    goals.observe_frontiers(0, 0.0)
    assert machine.state == MappingState.EXPLORING

    goals.observe_frontiers(0, 1.5)
    assert machine.state == MappingState.COMPLETION_CHECK

    assert not goals.confirm_map_complete(False)
    assert machine.state == MappingState.COMPLETION_CHECK

    cells = enclosed_map()
    verified, _ = verify_closed_free_region(
        cells, 10, 10, 1.0, min_free_area_m2=10.0
    )

    assert goals.confirm_map_complete(verified)
    assert machine.state == MappingState.MAP_COMPLETE
