"""Navigation utilities for AUV mission planning."""

import math
from typing import Tuple


def compute_heading(current_x: float, current_y: float,
                    target_x: float, target_y: float) -> float:
    """
    Compute heading from current position to target.

    Args:
        current_x, current_y: Current position in meters
        target_x, target_y: Target position in meters

    Returns:
        Heading in radians [-pi, pi], 0 = North/+X, pi/2 = East/+Y
    """
    dx = target_x - current_x
    dy = target_y - current_y
    return math.atan2(dy, dx)


def compute_distance(current_x: float, current_y: float,
                     target_x: float, target_y: float) -> float:
    """
    Compute Euclidean distance from current position to target.

    Args:
        current_x, current_y: Current position in meters
        target_x, target_y: Target position in meters

    Returns:
        Distance in meters
    """
    dx = target_x - current_x
    dy = target_y - current_y
    return math.sqrt(dx * dx + dy * dy)


def is_at_waypoint(current_x: float, current_y: float, current_depth: float,
                   target_x: float, target_y: float, target_depth: float,
                   xy_radius: float = 1.0, depth_tolerance: float = 0.5) -> bool:
    """
    Check if current position is within arrival radius of waypoint.

    Args:
        current_x, current_y, current_depth: Current position
        target_x, target_y, target_depth: Target waypoint
        xy_radius: Horizontal arrival radius in meters
        depth_tolerance: Vertical tolerance in meters

    Returns:
        True if within arrival radius
    """
    xy_dist = compute_distance(current_x, current_y, target_x, target_y)
    depth_diff = abs(current_depth - target_depth)
    return xy_dist <= xy_radius and depth_diff <= depth_tolerance


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi] range."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle
