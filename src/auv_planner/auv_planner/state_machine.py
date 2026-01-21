"""State machine for AUV mission control."""

from enum import Enum, auto
from typing import Callable, Optional
import time


class MissionState(Enum):
    """Mission states for autonomous operation."""
    IDLE = auto()
    DIVING = auto()
    NAVIGATING = auto()
    SURFACING = auto()
    COMPLETE = auto()
    ERROR = auto()


class MissionStateMachine:
    """
    State machine managing AUV mission phases.

    Transitions:
    - IDLE → DIVING: On start command
    - DIVING → NAVIGATING: When cruise depth reached
    - NAVIGATING → NAVIGATING: When waypoint reached (advance to next)
    - NAVIGATING → SURFACING: When all waypoints complete
    - SURFACING → COMPLETE: When surface reached
    - Any → ERROR: On error condition
    - ERROR → SURFACING: Automatic safe surfacing
    """

    def __init__(self, on_state_change: Optional[Callable[[MissionState, MissionState], None]] = None):
        """
        Initialize state machine.

        Args:
            on_state_change: Callback(old_state, new_state) when state changes
        """
        self._state = MissionState.IDLE
        self._on_state_change = on_state_change
        self._error_message = ""
        self._state_start_time = time.time()

    @property
    def state(self) -> MissionState:
        """Current state."""
        return self._state

    @property
    def error_message(self) -> str:
        """Error message if in ERROR state."""
        return self._error_message

    @property
    def time_in_state(self) -> float:
        """Seconds spent in current state."""
        return time.time() - self._state_start_time

    def _transition(self, new_state: MissionState):
        """Internal state transition."""
        old_state = self._state
        if old_state != new_state:
            self._state = new_state
            self._state_start_time = time.time()
            if self._on_state_change:
                self._on_state_change(old_state, new_state)

    def start_mission(self) -> bool:
        """
        Start the mission from IDLE state.

        Returns:
            True if transition successful
        """
        if self._state == MissionState.IDLE:
            self._transition(MissionState.DIVING)
            return True
        return False

    def depth_reached(self) -> bool:
        """
        Signal that cruise depth has been reached.

        Returns:
            True if transition successful
        """
        if self._state == MissionState.DIVING:
            self._transition(MissionState.NAVIGATING)
            return True
        return False

    def waypoints_complete(self) -> bool:
        """
        Signal that all waypoints have been visited.

        Returns:
            True if transition successful
        """
        if self._state == MissionState.NAVIGATING:
            self._transition(MissionState.SURFACING)
            return True
        return False

    def surface_reached(self) -> bool:
        """
        Signal that surface has been reached.

        Returns:
            True if transition successful
        """
        if self._state == MissionState.SURFACING:
            self._transition(MissionState.COMPLETE)
            return True
        # ERROR state auto-surfaces too
        if self._state == MissionState.ERROR:
            self._transition(MissionState.COMPLETE)
            return True
        return False

    def trigger_error(self, message: str = "Unknown error"):
        """
        Transition to ERROR state with safe surfacing.

        Args:
            message: Error description
        """
        if self._state not in (MissionState.COMPLETE, MissionState.ERROR):
            self._error_message = message
            self._transition(MissionState.ERROR)

    def reset(self):
        """Reset state machine to IDLE."""
        self._transition(MissionState.IDLE)
        self._error_message = ""

    def is_active(self) -> bool:
        """Check if mission is actively running (not IDLE or COMPLETE)."""
        return self._state not in (MissionState.IDLE, MissionState.COMPLETE)
