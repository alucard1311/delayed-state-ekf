"""Waypoint management for AUV missions."""

from dataclasses import dataclass
from typing import List, Optional
import yaml


@dataclass
class Waypoint:
    """Single waypoint with x, y, depth coordinates."""
    x: float
    y: float
    depth: float
    name: str = ""


class WaypointManager:
    """Manages waypoint sequence for missions."""

    def __init__(self):
        self._waypoints: List[Waypoint] = []
        self._current_index: int = 0

    def load_from_yaml(self, yaml_path: str) -> bool:
        """
        Load waypoints from YAML file.

        Expected format:
        waypoints:
          - {x: 5.0, y: 0.0, depth: 3.0, name: "WP1"}
          - {x: 5.0, y: 5.0, depth: 3.0, name: "WP2"}
        """
        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)

            self._waypoints = []
            for wp_data in data.get('waypoints', []):
                wp = Waypoint(
                    x=float(wp_data['x']),
                    y=float(wp_data['y']),
                    depth=float(wp_data['depth']),
                    name=wp_data.get('name', f"WP{len(self._waypoints)}")
                )
                self._waypoints.append(wp)

            self._current_index = 0
            return len(self._waypoints) > 0
        except Exception as e:
            print(f"Error loading waypoints: {e}")
            return False

    def get_current_waypoint(self) -> Optional[Waypoint]:
        """Get current target waypoint, or None if complete."""
        if self._current_index < len(self._waypoints):
            return self._waypoints[self._current_index]
        return None

    def advance_waypoint(self) -> bool:
        """
        Advance to next waypoint.

        Returns:
            True if advanced to next waypoint, False if sequence complete
        """
        if self._current_index < len(self._waypoints) - 1:
            self._current_index += 1
            return True
        return False

    def reset(self):
        """Reset to first waypoint."""
        self._current_index = 0

    def is_complete(self) -> bool:
        """Check if all waypoints have been visited."""
        return self._current_index >= len(self._waypoints)

    @property
    def total_waypoints(self) -> int:
        """Total number of waypoints."""
        return len(self._waypoints)

    @property
    def current_index(self) -> int:
        """Current waypoint index (0-based)."""
        return self._current_index
