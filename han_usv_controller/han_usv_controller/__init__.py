"""VRX WAM-V closed-loop autonomous controller."""

from .core import (
    AvoidanceDecision,
    ControlConfig,
    ControllerCore,
    GeoTarget,
    PIDController,
    extract_obstacle_points,
    nearest_neighbor_order,
)

__all__ = [
    'AvoidanceDecision',
    'ControlConfig',
    'ControllerCore',
    'GeoTarget',
    'PIDController',
    'extract_obstacle_points',
    'nearest_neighbor_order',
]
