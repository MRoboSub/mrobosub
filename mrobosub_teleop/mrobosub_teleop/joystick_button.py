"""
Button helpers for joystick teleop.

Provides Button (rising-edge detection and press tracking) used by
joystick_teleop_continuous for discrete button actions.
"""

KNOWN_ACTIONS = {"estop", "zero_state"}


class Button:
    """Tracks press state for a single joystick button."""

    def __init__(self, idx: int, action: str) -> None:
        if action not in KNOWN_ACTIONS:
            raise ValueError(f"Unknown button action '{action}'. Known: {KNOWN_ACTIONS}")
        self.idx = idx
        self.name = action
        self._is_pressed = False
        self._was_pressed = False

    def update(self, is_pressed: bool) -> None:
        """Update pressed state, preserving previous state for edge detection."""
        self._was_pressed = self._is_pressed
        self._is_pressed = is_pressed

    @property
    def just_pressed(self) -> bool:
        """True only on the rising edge (not-pressed -> pressed transition)."""
        return self._is_pressed and not self._was_pressed
