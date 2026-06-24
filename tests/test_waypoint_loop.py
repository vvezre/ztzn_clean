from itertools import islice

import pytest

from waypoint_loop import (
    build_closed_loop_targets,
    iter_closed_loop_targets,
    normalize_loop_options,
)


WAYPOINTS = [
    {"lat": 1.0, "lon": 1.0},
    {"lat": 2.0, "lon": 2.0},
    {"lat": 3.0, "lon": 3.0},
    {"lat": 4.0, "lon": 4.0},
]


def test_closed_loop_targets_approach_first_point_then_close_each_cycle():
    targets = build_closed_loop_targets(WAYPOINTS, loop_count=2)

    assert [target["index"] for target in targets] == [
        0,
        1, 2, 3, 0,
        1, 2, 3, 0,
    ]
    assert [target["completed_loop"] for target in targets] == [
        0,
        0, 0, 0, 1,
        1, 1, 1, 2,
    ]


def test_continuous_loop_targets_repeat_after_each_return_to_first_point():
    targets = list(islice(iter_closed_loop_targets(WAYPOINTS), 9))

    assert [target["index"] for target in targets] == [
        0,
        1, 2, 3, 0,
        1, 2, 3, 0,
    ]
    assert targets[-1]["completed_loop"] == 2


def test_normalize_count_loop_requires_positive_count_and_two_points():
    with pytest.raises(ValueError, match="至少需要2个路点"):
        normalize_loop_options(
            {"loop": True, "loopMode": "count", "loopCount": 2},
            waypoint_count=1,
        )

    with pytest.raises(ValueError, match="循环圈数"):
        normalize_loop_options(
            {"loop": True, "loopMode": "count", "loopCount": 0},
            waypoint_count=2,
        )


def test_normalize_continuous_loop_uses_zero_target():
    options = normalize_loop_options(
        {"loop": True, "loopMode": "continuous", "loopCount": 99},
        waypoint_count=2,
    )

    assert options == {
        "loop": True,
        "loopMode": "continuous",
        "loopCount": 0,
    }


def test_normalize_missing_options_preserves_single_pass_compatibility():
    options = normalize_loop_options({}, waypoint_count=1)

    assert options == {
        "loop": False,
        "loopMode": "count",
        "loopCount": 1,
    }
