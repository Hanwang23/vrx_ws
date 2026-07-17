import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from codex_usv_controller.debug_visualization import (
    TrackingStatistics,
    circle_points,
    enu_history_to_body,
    enu_offset_to_body,
    filter_buoy_candidates,
    freshness_state,
    stale_buoy_marker_ids,
    tracking_quality,
    tracking_statistics,
    waypoint_visual_state,
)
from codex_usv_controller.node import AutonomousUSVNode


def test_enu_history_rotates_into_current_body_frame():
    points = enu_history_to_body(
        ((8.0, 20.0), (10.0, 20.0)),
        current_east=10.0,
        current_north=20.0,
        current_yaw=math.pi / 2.0,
    )
    assert points[0][0] == pytest.approx(0.0)
    assert points[0][1] == pytest.approx(2.0)
    assert points[1] == pytest.approx((0.0, 0.0))


def test_enu_offset_and_circle_geometry_are_stable():
    assert enu_offset_to_body(0.0, 4.0, math.pi / 2.0) == pytest.approx(
        (4.0, 0.0))
    points = circle_points(5.5, sample_count=12)
    assert len(points) == 13
    assert points[0] == pytest.approx(points[-1])
    assert all(math.hypot(x, y) == pytest.approx(5.5) for x, y in points)


def test_waypoint_state_and_sensor_freshness_are_unambiguous():
    assert waypoint_visual_state(1, 2).label == 'DONE'
    assert waypoint_visual_state(2, 2).label == 'CURRENT'
    assert waypoint_visual_state(3, 2).label == 'PENDING'
    assert freshness_state(None, 1.0) == 'WAIT'
    assert freshness_state(0.5, 1.0) == 'OK'
    assert freshness_state(1.1, 1.0) == 'STALE'


def test_tracking_statistics_excludes_old_and_safety_samples():
    statistics = tracking_statistics((
        (70.0, 9.0, False),
        (90.0, 5.0, True),
        (95.0, -0.5, False),
        (100.0, 1.5, False),
    ), now=100.0, window_s=20.0)
    assert statistics.sample_count == 2
    assert statistics.mean_abs_m == 1.0
    assert statistics.max_abs_m == 1.5


def test_tracking_quality_distinguishes_accuracy_and_safety():
    good = TrackingStatistics(0.7, 1.1, 10)
    assert tracking_quality(True, 0.8, good, False).label == 'ON TRACK'
    assert tracking_quality(True, 4.0, good, True).label == 'SAFETY MANEUVER'
    assert tracking_quality(True, 4.0, good, False).label == 'OFF PATH'
    assert tracking_quality(
        True, 4.0, good, False, terminal_active=True,
    ).label == 'GOAL APPROACH'
    assert tracking_quality(False, 0.0, good, False).label == 'NO ACTIVE PATH'


def test_buoy_candidates_exclude_dedicated_dynamic_vessels():
    candidates = filter_buoy_candidates(
        ((10.0, 0.0), (12.0, math.pi / 2.0)),
        ((11.5, 0.0),),
        exclusion_radius=2.0,
        sensor_offset=(1.5, 0.0),
    )
    assert candidates == ((12.0, math.pi / 2.0),)


def test_debug_view_does_not_duplicate_candidates_as_red_obstacles():
    source = (
        Path(__file__).parents[1] / 'codex_usv_controller' / 'node.py'
    ).read_text(encoding='utf-8')
    assert "'tracked_obstacles'" not in source
    assert "label.text = f'BUOY #" in source


def test_richer_debug_view_is_rate_limited_and_keeps_task_layers():
    source = (
        Path(__file__).parents[1] / 'codex_usv_controller' / 'node.py'
    ).read_text(encoding='utf-8')
    assert "'debug.publish_rate_hz': 5.0" in source
    assert "marker('mission_route'" in source
    assert "'mission_waypoints'" in source
    assert "'navigation_limits'" in source
    assert "'motion_vectors'" in source
    assert "'sensor_health'" in source
    assert "'planner_health'" in source


def test_buoy_markers_delete_only_stale_ids_without_global_clear():
    assert stale_buoy_marker_ids(3, 1) == (20, 21, 22, 30, 31, 32)
    assert stale_buoy_marker_ids(1, 3) == ()
    source = (
        Path(__file__).parents[1] / 'codex_usv_controller' / 'node.py'
    ).read_text(encoding='utf-8')
    assert 'Marker.DELETEALL' not in source
    assert 'stale.action = Marker.DELETE' in source


def test_debug_history_restarts_when_path_revision_changes():
    fake_node = SimpleNamespace(
        navigation_east=12.0,
        navigation_north=5.0,
        trajectory_history=[(1.0, 1.0), (2.0, 2.0)],
        tracking_error_history=[(9.0, 4.0, False)],
        debug_path_revision=3,
    )
    command = SimpleNamespace(
        path_valid=True,
        path_revision=4,
        state='navigating',
        cross_track_error=0.4,
        avoidance_override=False,
        avoidance_episode_active=False,
        colregs_active=False,
    )
    AutonomousUSVNode._update_debug_history(fake_node, 10.0, command)
    assert fake_node.debug_path_revision == 4
    assert fake_node.trajectory_history == [(12.0, 5.0)]
    assert fake_node.tracking_error_history == [(10.0, 0.4, False)]
