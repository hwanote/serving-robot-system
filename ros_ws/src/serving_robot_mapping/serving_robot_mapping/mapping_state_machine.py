"""State machine definitions for autonomous store mapping."""

from enum import Enum


class MappingState(str, Enum):
    """High-level autonomous mapping states."""

    IDLE = "IDLE"
    INITIAL_SCAN = "INITIAL_SCAN"
    EXPLORING = "EXPLORING"
    EXPLORATION_BLOCKED = "EXPLORATION_BLOCKED"
    NAVIGATING = "NAVIGATING"
    RECOVERY = "RECOVERY"
    COMPLETION_CHECK = "COMPLETION_CHECK"
    MAP_COMPLETE = "MAP_COMPLETE"
    ERROR = "ERROR"


class MappingStateMachine:
    """Minimal state holder for the autonomous mapping workflow."""

    def __init__(self) -> None:
        self._state = MappingState.IDLE

    @property
    def state(self) -> MappingState:
        """Return the current mapping state."""
        return self._state

    def transition_to(self, new_state: MappingState) -> None:
        """Transition to a new mapping state."""
        self._state = new_state
