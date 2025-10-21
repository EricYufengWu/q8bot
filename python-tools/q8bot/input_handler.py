'''
Written by yufeng.wu0902@gmail.com

Unified input handler for Q8bot control.
Abstracts keyboard and joystick input into a common interface.
'''

import pygame
from control_config import (
    KEYBOARD_MAPPING,
    check_joystick_compatible,
    get_joystick_mapping,
    apply_deadzone,
    get_joystick_direction
)
from helpers import Q8Logger


class InputHandler:
    """Unified input handler for both keyboard and joystick."""

    def __init__(self, use_joystick=False, joystick=None, joystick_mapping=None):
        """
        Initialize input handler.

        Args:
            use_joystick: bool, True to use joystick mode
            joystick: pygame.joystick.Joystick object (if using joystick)
            joystick_mapping: dict from get_joystick_mapping() (if using joystick)
        """
        self.use_joystick = use_joystick
        self.joystick = joystick
        self.joystick_mapping = joystick_mapping

    def get_movement_direction(self):
        """
        Get current movement direction from keyboard or joystick.

        Returns:
            str: Direction command string or None if no input
                For joystick: Can return 12 different commands (f, b, l, r, fl_0.75, fl_0.5, etc.)
                For keyboard: Returns 6 basic commands (f, b, l, r, fl, fr)
        """
        if self.use_joystick:
            pygame.event.pump()
            axes = self.joystick_mapping['axes'] if self.joystick_mapping else {
                'horizontal': 0, 'vertical': 1, 'deadzone': 0.1
            }
            deadzone = axes.get('deadzone', 0.1)

            axis0 = apply_deadzone(self.joystick.get_axis(axes['horizontal']), deadzone)
            axis1 = apply_deadzone(self.joystick.get_axis(axes['vertical']), deadzone)

            # Use analog mode for joystick to get full range of commands
            return get_joystick_direction(axis0, axis1, analog_mode=True)
        else:
            # Keyboard mode - map to basic commands
            keys = pygame.key.get_pressed()
            # Map keyboard to closest trajectory equivalents
            if keys[KEYBOARD_MAPPING['movement']['forward']]:
                return 'f'
            elif keys[KEYBOARD_MAPPING['movement']['backward']]:
                return 'b'
            elif keys[KEYBOARD_MAPPING['movement']['left']]:
                return 'l'
            elif keys[KEYBOARD_MAPPING['movement']['right']]:
                return 'r'
            elif keys[KEYBOARD_MAPPING['movement']['forward_left']]:
                return 'fl_0.75'  # Map to moderate forward-left turn
            elif keys[KEYBOARD_MAPPING['movement']['forward_right']]:
                return 'fr_0.75'  # Map to moderate forward-right turn
            return None

    def is_movement_input(self):
        """
        Check if any movement input is active.

        Returns:
            bool: True if movement input detected
        """
        return self.get_movement_direction() is not None

    def is_action_pressed(self, action_name):
        """
        Check if a specific action button/key is pressed.

        Args:
            action_name: str, action name ('greet', 'battery', 'switch_gait', etc.)

        Returns:
            bool: True if action is pressed
        """
        if self.use_joystick:
            pygame.event.pump()
            if self.joystick_mapping and action_name in self.joystick_mapping['actions']:
                button_num = self.joystick_mapping['actions'][action_name]
                return self.joystick.get_button(button_num)
            return False
        else:
            # Keyboard mode
            keys = pygame.key.get_pressed()
            if action_name in KEYBOARD_MAPPING['actions']:
                return keys[KEYBOARD_MAPPING['actions'][action_name]]
            return False


def detect_and_init_joystick():
    """
    Detect and initialize compatible joystick.
    Falls back to keyboard if no compatible joystick found.

    Returns:
        tuple: (use_joystick, joystick, joystick_mapping)
            - use_joystick: bool, True if joystick should be used
            - joystick: pygame.joystick.Joystick object or None
            - joystick_mapping: dict from get_joystick_mapping() or None
    """
    pygame.joystick.init()
    joystick_count = pygame.joystick.get_count()

    Q8Logger.debug("=" * 60)
    Q8Logger.debug(f"Detected {joystick_count} joystick(s)")

    if joystick_count > 0:
        # Try each joystick until we find a compatible one
        for i in range(joystick_count):
            js = pygame.joystick.Joystick(i)
            js.init()
            Q8Logger.debug(f"\nJoystick {i}: {js.get_name()}")
            Q8Logger.debug(f"  Axes: {js.get_numaxes()}, Buttons: {js.get_numbuttons()}")

            if check_joystick_compatible(js):
                joystick_mapping = get_joystick_mapping(js)
                Q8Logger.debug(f"\n>>> Using joystick: '{js.get_name()}' <<<")

                if joystick_mapping:
                    if joystick_mapping['recognized']:
                        Q8Logger.debug(">>> Controller is RECOGNIZED (custom mapping loaded) <<<")
                    else:
                        Q8Logger.warning("Controller not recognized - using GPDWIN fallback mapping")
                else:
                    Q8Logger.warning("Error loading joystick mapping")
                    continue

                Q8Logger.debug("=" * 60)
                return True, js, joystick_mapping

    Q8Logger.debug("\n>>> No compatible joystick found - using KEYBOARD control <<<")
    Q8Logger.debug("=" * 60)
    return False, None, None
