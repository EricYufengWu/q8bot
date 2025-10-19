'''
Written by yufeng.wu0902@gmail.com

Control mappings configuration for Q8bot.
Defines keyboard and joystick control schemes using a two-layer mapping system.
'''

import pygame
import json
import os

# =============================================================================
# KEYBOARD CONFIGURATION
# =============================================================================

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
        'greet': pygame.K_h,
        'battery': pygame.K_b,
        'switch_gait': pygame.K_g,
        'jump': pygame.K_j,
        'reset': pygame.K_r,
        'exit': pygame.K_ESCAPE,
        'record': pygame.K_z,
        'show_range': pygame.K_c,
    }
}

# =============================================================================
# JOYSTICK CONFIGURATION
# =============================================================================

def load_joystick_config():
    """
    Load joystick configuration from joystick_config.json.

    Returns:
        dict: Joystick configuration or None if loading failed
    """
    config_path = os.path.join(os.path.dirname(__file__), 'joystick_config.json')
    try:
        with open(config_path, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"Warning: joystick_config.json not found at {config_path}")
        return None
    except json.JSONDecodeError as e:
        print(f"Error parsing joystick_config.json: {e}")
        return None


# Global joystick configuration (loaded from JSON)
JOYSTICK_CONFIG = load_joystick_config()

# Movement control settings (universal for all joysticks)
JOYSTICK_MOVEMENT = {
    'forward_backward_axis': 1,      # axis[1]: -1 = forward, +1 = backward
    'left_right_axis': 0,             # axis[0]: -1 = left, +1 = right
    'analog_mode': True,              # Analog stride control enabled
}

# =============================================================================
# JOYSTICK HELPER FUNCTIONS
# =============================================================================

def check_joystick_compatible(joystick):
    """
    Check if the connected joystick is compatible.
    Returns True if joystick is recognized OR meets minimum requirements.

    Args:
        joystick: pygame.joystick.Joystick object

    Returns:
        bool: True if joystick is compatible, False otherwise
    """
    if joystick is None or JOYSTICK_CONFIG is None:
        return False

    joystick_name = joystick.get_name()
    num_axes = joystick.get_numaxes()
    num_buttons = joystick.get_numbuttons()

    # Check if joystick is in our recognized list
    if joystick_name in JOYSTICK_CONFIG['controllers']:
        print(f"Recognized joystick: '{joystick_name}'")
        return True

    # Fall back to minimum requirements check
    required_axes = JOYSTICK_CONFIG['requirements']['min_axes']
    required_buttons = JOYSTICK_CONFIG['requirements']['min_buttons']

    if num_axes < required_axes or num_buttons < required_buttons:
        print(f"Warning: Joystick '{joystick_name}' does not meet requirements.")
        print(f"  Required: {required_axes} axes, {required_buttons} buttons")
        print(f"  Found: {num_axes} axes, {num_buttons} buttons")
        return False

    print(f"Joystick '{joystick_name}' meets minimum requirements (generic mode)")
    return True


def get_joystick_mapping(joystick):
    """
    Get the button/axis mapping for a specific joystick using two-layer lookup.

    Layer 1: Action → Physical Position (e.g., 'greet' → 'top')
    Layer 2: Physical Position → Button Number (e.g., 'top' → 2 for Joycon, 0 for GPDWIN)

    If joystick is not recognized but meets minimum requirements, falls back to GPDWIN mapping.

    Args:
        joystick: pygame.joystick.Joystick object

    Returns:
        dict: Joystick mapping with structure:
            {
                'name': str,
                'axes': dict,
                'buttons': dict,      # Physical position → button number
                'actions': dict,      # Action → button number (resolved)
                'recognized': bool
            }
        None if joystick not supported or config not loaded
    """
    if joystick is None or JOYSTICK_CONFIG is None:
        return None

    joystick_name = joystick.get_name()
    controller_config = None
    recognized = False

    # Check if we have a config for this specific controller
    if joystick_name in JOYSTICK_CONFIG['controllers']:
        controller_config = JOYSTICK_CONFIG['controllers'][joystick_name]
        recognized = True
    else:
        # Fall back to GPDWIN mapping for unrecognized controllers
        if 'GPD WIN Game Controller' in JOYSTICK_CONFIG['controllers']:
            controller_config = JOYSTICK_CONFIG['controllers']['GPD WIN Game Controller']
            recognized = False  # Using generic fallback
        else:
            return None

    # Build action-to-button mapping using two-layer lookup
    action_buttons = {}
    for action, position in JOYSTICK_CONFIG['action_mapping'].items():
        if position in controller_config['buttons']:
            action_buttons[action] = controller_config['buttons'][position]

    return {
        'name': joystick_name,
        'axes': controller_config['axes'],
        'buttons': controller_config['buttons'],  # Physical position mapping
        'actions': action_buttons,                # Logical action mapping
        'recognized': recognized
    }


def apply_deadzone(value, deadzone):
    """
    Apply deadzone to joystick axis value.

    Args:
        value: float, axis value
        deadzone: float, deadzone threshold

    Returns:
        float: 0.0 if within deadzone, otherwise original value
    """
    return 0.0 if abs(value) < deadzone else value


def get_joystick_direction(axis0, axis1, analog_mode=False):
    """
    Determine movement direction from joystick axes.
    Returns recognized command strings matching trajectory names.

    NOTE: axis0 is left/right, axis1 is forward/backward

    Mapping scheme (analog mode):
    - Straight: abs(a[0]) < 0.25 → 'f' or 'b'
    - Moderate turn: 0.25 <= abs(a[0]) < 0.6 → 'fl_0.75', 'fr_0.75', 'bl_0.75', 'br_0.75'
    - Strong turn: 0.6 <= abs(a[0]) < 0.85 → 'fl_0.5', 'fr_0.5', 'bl_0.5', 'br_0.5'
    - Full left/right: abs(a[0]) >= 0.85 and abs(a[1]) <= 0.4 → 'l', 'r'

    Args:
        axis0: float, left/right axis value (negative = left, positive = right)
        axis1: float, forward/backward axis value (negative = forward, positive = backward)
        analog_mode: bool, if True returns command strings, else simple directions

    Returns:
        str: Command string ('f', 'b', 'l', 'r', 'fl_0.75', 'fl_0.5', etc.) or None
    """
    if not analog_mode:
        # Binary mode: simple 4-direction control
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
        # Analog mode: returns command strings matching trajectory names
        # Check for no input
        if axis0 == 0 and axis1 == 0:
            return None

        abs_a0 = abs(axis0)
        abs_a1 = abs(axis1)

        # Full turn mode: high horizontal, low vertical (tightened window)
        if abs_a0 >= 0.85 and abs_a1 <= 0.4:
            # Only very extreme positions are pure left/right
            return 'l' if axis0 < 0 else 'r'

        # Forward/backward with turning
        if abs_a1 > abs_a0:  # Forward/backward dominant
            base_dir = 'f' if axis1 < 0 else 'b'
            turn_dir = 'l' if axis0 < 0 else 'r'

            # Determine turn intensity (expanded 0.5 window)
            if abs_a0 < 0.25:  # Straight movement
                return base_dir
            elif abs_a0 < 0.6:  # Moderate turn (75% stride)
                return f"{base_dir}{turn_dir}_0.75"
            else:  # Strong turn (50% stride) - now abs_a0 >= 0.6
                return f"{base_dir}{turn_dir}_0.5"
        else:
            # Turning is dominant - use full turn mode
            return 'l' if axis0 < 0 else 'r'
