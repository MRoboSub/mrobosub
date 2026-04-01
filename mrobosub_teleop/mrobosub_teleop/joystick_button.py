"""
Button class just wraps button state logic
"""

from enum import Enum


class ButtonAction(str, Enum):
    """Valid ``use`` strings from YAML; str mixin keeps values JSON/YAML-friendly."""

    ESTOP = "estop"
    ZERO_STATE = "zero_state"


class Button:
    def __init__(self, idx: int, action: str | ButtonAction) -> None:
        self.idx = idx
        self.action = (
            action if isinstance(action, ButtonAction) else ButtonAction(action)
        )
        self.is_pressed = False
        self.was_pressed = False

    @property
    def name(self) -> str:
        return self.action.value

    def update(self, is_pressed: bool) -> None:
        """Update pressed state, preserving previous state for edge detection."""
        self.was_pressed = self.is_pressed
        self.is_pressed = is_pressed

    @property
    def just_pressed(self) -> bool:
        """True only on the transition to pressed (rising edge), not while held."""
        return self.is_pressed and not self.was_pressed
