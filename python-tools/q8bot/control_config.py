'''
Written by yufeng.wu0902@gmail.com

Control mappings configuration for Q8bot.
Defines keyboard and joystick control schemes.
'''

import pygame

# Keyboard control mapping (default)
KEYBOARD_MAPPING = {
    'movement': {
        'forward': pygame.K_w,
        'backward': pygame.K_s,
        'left': pygame.K_a,
        'right': pygame.K_d,
        'forward_left': pygame.K_q,
        'forward_right': pygame.K_e,
    },
    'actions': {
        'reset': pygame.K_r,
        'jump': pygame.K_j,
        'switch_gait': pygame.K_g,
        'battery': pygame.K_b,
        'record': pygame.K_z,
        'show_range': pygame.K_c,
        'greet': pygame.K_h,
        'exit': pygame.K_ESCAPE,
    }
}

# Joystick control mapping
JOYSTICK_MAPPING = {
    'requirements': {
        'min_axes': 2,
        'min_buttons': 5,
    },
    'movement': {
        'forward_backward_axis': 1,      # axis[1]: -1 = forward, +1 = backward
        'left_right_axis': 0,             # axis[0]: -1 = left, +1 = right
        'deadzone': 0.01,
        'analog_mode': True,              # Analog stride control enabled
    },
    'actions': {
        'greet': 0,          # button[0]
        'battery': 1,        # button[1]
        'switch_gait': 2,    # button[2]
        'jump': 3,           # button[3]
        'reset': 4,          # button[4]
    }
}

def check_joystick_compatible(joystick):
    """
    Check if the connected joystick meets the requirements.

    Args:
        joystick: pygame.joystick.Joystick object

    Returns:
        bool: True if joystick is compatible, False otherwise
    """
    if joystick is None:
        return False

    num_axes = joystick.get_numaxes()
    num_buttons = joystick.get_numbuttons()

    required_axes = JOYSTICK_MAPPING['requirements']['min_axes']
    required_buttons = JOYSTICK_MAPPING['requirements']['min_buttons']

    if num_axes < required_axes or num_buttons < required_buttons:
        print(f"Warning: Joystick '{joystick.get_name()}' does not meet requirements.")
        print(f"  Required: {required_axes} axes, {required_buttons} buttons")
        print(f"  Found: {num_axes} axes, {num_buttons} buttons")
        return False

    return True

def apply_deadzone(value, deadzone):
    """
    Apply deadzone to analog input.

    Args:
        value: float, analog input value
        deadzone: float, deadzone threshold

    Returns:
        float: 0.0 if within deadzone, original value otherwise
    """
    return 0.0 if abs(value) < deadzone else value

def get_joystick_direction(axis0, axis1, analog_mode=False):
    """
    Determine movement direction and stride percentages from joystick axes.

    NOTE: axis0 is left/right, axis1 is forward/backward

    Mapping scheme:
    - a[1] = -1, -0.25 < a[0] < 0.25: straight forward, both sides 100%
    - a[1] = -1, -0.75 < a[0] < -0.25: forward-left, right 100%, left 75%
    - -1 <= a[1] < -0.75, -1 <= a[0] < -0.75: forward-left, right 100%, left 50%
    - -0.75 < a[1] < -0.25, a[0] = -1: forward-left, right 100%, left 25%
    - -0.25 < a[1] < 0.25, a[0] = -1: full left turn mode

    Args:
        axis0: float, left/right axis value (negative = left, positive = right)
        axis1: float, forward/backward axis value (negative = forward, positive = backward)
        analog_mode: bool, if True uses analog stride control

    Returns:
        If analog_mode=False: direction string ('f', 'b', 'l', 'r', None)
        If analog_mode=True: tuple (direction, stride_left, stride_right)
    """
    if not analog_mode:
        # Binary mode: priority given to axis1 (forward/backward), then axis0 (turning)
        if axis1 < 0:
            return 'f'  # Forward
        elif axis1 > 0:
            return 'b'  # Backward
        elif axis0 < 0:
            return 'l'  # Left turn
        elif axis0 > 0:
            return 'r'  # Right turn
        else:
            return None
    else:
        # Analog mode with variable stride control
        import math

        # Check for no input
        if axis0 == 0 and axis1 == 0:
            return None, 1.0, 1.0

        # Determine if primarily forward/backward or turning
        abs_a0 = abs(axis0)
        abs_a1 = abs(axis1)

        # Full turn mode: mostly horizontal, minimal vertical
        if abs_a0 >= 0.75 and abs_a1 <= 0.25:
            if axis0 < 0:
                return 'l', 1.0, 1.0  # Full left turn
            else:
                return 'r', 1.0, 1.0  # Full right turn

        # Forward/backward with turning
        if abs_a1 > abs_a0:  # Forward/backward dominant
            direction = 'f' if axis1 < 0 else 'b'

            # Determine stride reduction based on axis0 (left/right)
            if abs_a0 < 0.25:  # Straight
                stride_l, stride_r = 1.0, 1.0
            elif abs_a0 < 0.75:  # Moderate turn
                if axis0 < 0:  # Turning left
                    stride_l, stride_r = 0.75, 1.0
                else:  # Turning right
                    stride_l, stride_r = 1.0, 0.75
            else:  # Strong turn (abs_a0 >= 0.75)
                if abs_a1 >= 0.75:  # Strong forward + strong turn = 50% reduction
                    if axis0 < 0:
                        stride_l, stride_r = 0.5, 1.0
                    else:
                        stride_l, stride_r = 1.0, 0.5
                else:  # Weak forward + strong turn = 25% reduction
                    if axis0 < 0:
                        stride_l, stride_r = 0.25, 1.0
                    else:
                        stride_l, stride_r = 1.0, 0.25

            return direction, stride_l, stride_r
        else:
            # Turning is dominant - use full turn mode
            if axis0 < 0:
                return 'l', 1.0, 1.0
            else:
                return 'r', 1.0, 1.0
