'''
Written by yufeng.wu0902@gmail.com

Test script to verify control_config.py keyboard and joystick mappings.
Prints movement and action states in real-time.
'''

import pygame
import sys
from control_config import (
    KEYBOARD_MAPPING,
    JOYSTICK_MOVEMENT,
    check_joystick_compatible,
    get_joystick_mapping,
    apply_deadzone,
    get_joystick_direction
)

def main():
    pygame.init()
    pygame.joystick.init()

    # Create a small window (required for pygame event loop)
    window = pygame.display.set_mode((400, 300))
    pygame.display.set_caption("Control Config Test")
    clock = pygame.time.Clock()

    # Check for joystick
    use_joystick = False
    joystick = None
    joystick_mapping = None

    joystick_count = pygame.joystick.get_count()
    print("=" * 60)
    print(f"Detected {joystick_count} joystick(s)")

    if joystick_count > 0:
        # Try each joystick until we find a compatible one
        for i in range(joystick_count):
            js = pygame.joystick.Joystick(i)
            js.init()
            print(f"\nJoystick {i}: {js.get_name()}")
            print(f"  Axes: {js.get_numaxes()}, Buttons: {js.get_numbuttons()}")

            if check_joystick_compatible(js):
                joystick = js
                joystick_mapping = get_joystick_mapping(js)
                use_joystick = True
                print(f"\n>>> Using joystick: '{js.get_name()}' <<<")

                if joystick_mapping:
                    if joystick_mapping['recognized']:
                        print(">>> Controller is RECOGNIZED (custom mapping loaded) <<<")
                    else:
                        print(">>> Controller not recognized - using GPDWIN fallback mapping <<<")

                    print("\nAction Mappings:")
                    for action, button in joystick_mapping['actions'].items():
                        # Find physical position for this button
                        position = None
                        for pos, btn in joystick_mapping['buttons'].items():
                            if btn == button:
                                position = pos
                                break
                        print(f"  {action:15} -> button {button:2} ({position})")
                else:
                    print(">>> Error loading joystick mapping <<<")
                break

    if not use_joystick:
        print("\n>>> Falling back to KEYBOARD control <<<")
        print("\nKeyboard Mappings:")
        print("Movement:")
        for action, key in KEYBOARD_MAPPING['movement'].items():
            print(f"  {action:15} -> {pygame.key.name(key)}")
        print("Actions:")
        for action, key in KEYBOARD_MAPPING['actions'].items():
            print(f"  {action:15} -> {pygame.key.name(key)}")

    print("=" * 60)
    print("\nPress ESC to exit")
    print("=" * 60)
    print()

    # State tracking
    last_movement = None
    last_actions = set()

    # Main loop
    running = True
    while running:
        clock.tick(60)  # 60 FPS

        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Clear previous state
        current_movement = None
        current_actions = set()

        if use_joystick:
            # ===== JOYSTICK MODE =====
            pygame.event.pump()  # Update joystick state

            # Get axis values
            axis_config = joystick_mapping['axes'] if joystick_mapping else {'horizontal': 0, 'vertical': 1, 'deadzone': 0.1}
            deadzone = axis_config.get('deadzone', 0.1)

            axis0 = apply_deadzone(joystick.get_axis(axis_config['horizontal']), deadzone)
            axis1 = apply_deadzone(joystick.get_axis(axis_config['vertical']), deadzone)

            # Determine movement direction
            direction = get_joystick_direction(axis0, axis1, analog_mode=False)
            if direction:
                current_movement = direction

            # Check action buttons
            if joystick_mapping and joystick_mapping['recognized']:
                # Use recognized controller's action mapping
                for action, button_num in joystick_mapping['actions'].items():
                    if joystick.get_button(button_num):
                        current_actions.add(action)
            else:
                # Generic mode: just check first 6 buttons
                action_names = ['greet', 'battery', 'switch_gait', 'jump', 'reset', 'exit']
                for i, action in enumerate(action_names):
                    if i < joystick.get_numbuttons() and joystick.get_button(i):
                        current_actions.add(action)

        else:
            # ===== KEYBOARD MODE =====
            keys = pygame.key.get_pressed()

            # Check movement keys (priority: forward/backward > left/right)
            if keys[KEYBOARD_MAPPING['movement']['forward']]:
                current_movement = 'forward'
            elif keys[KEYBOARD_MAPPING['movement']['backward']]:
                current_movement = 'backward'
            elif keys[KEYBOARD_MAPPING['movement']['left']]:
                current_movement = 'left'
            elif keys[KEYBOARD_MAPPING['movement']['right']]:
                current_movement = 'right'
            elif keys[KEYBOARD_MAPPING['movement']['forward_left']]:
                current_movement = 'forward_left'
            elif keys[KEYBOARD_MAPPING['movement']['forward_right']]:
                current_movement = 'forward_right'

            # Check action keys
            for action, key in KEYBOARD_MAPPING['actions'].items():
                if keys[key]:
                    current_actions.add(action)
                    if action == 'exit':
                        running = False

        # Print state changes
        if current_movement != last_movement:
            if current_movement:
                print(f"Movement: {current_movement}")
            else:
                print("Movement: IDLE")
            last_movement = current_movement

        if current_actions != last_actions:
            if current_actions:
                print(f"Actions:  {', '.join(sorted(current_actions))}")
            else:
                print("Actions:  NONE")
            last_actions = current_actions.copy()

    # Cleanup
    if joystick:
        joystick.quit()
    pygame.quit()
    print("\nTest ended.")


if __name__ == "__main__":
    main()
