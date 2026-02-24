"""
Button helpers for joystick teleop.

Provides Button (rising-edge detection and press tracking) used by
joystick_teleop_continuous for discrete button actions.
"""

KNOWN_ACTIONS = {"estop", "switch"}


class Button:
    """Tracks press state for a single joystick button."""

    def __init__(self, idx: int, action: str) -> None:
        if action not in KNOWN_ACTIONS:
            raise ValueError(f"Unknown button action '{action}'. Known: {KNOWN_ACTIONS}")
        self.idx = idx
        self.name = action
        self._is_pressed = False

    def update(self, is_pressed: bool) -> None:
        """Update pressed state."""
        self._is_pressed = is_pressed

    @property
    def pressed(self) -> bool:
        return self._is_pressed
