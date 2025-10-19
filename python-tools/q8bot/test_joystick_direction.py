'''
Written by yufeng.wu0902@gmail.com

Test script for get_joystick_direction() logic.
Interactive pygame loop that shows real-time joystick direction detection.
'''

import pygame
import sys
from control_config import get_joystick_direction, apply_deadzone

def test_joystick_direction():
    """Test get_joystick_direction with various axis inputs."""

    print("=" * 70)
    print("Testing get_joystick_direction() - Analog Mode")
    print("=" * 70)
    print()

    # Test cases: (axis0, axis1, expected_output, description)
    test_cases = [
        # No input
        (0.0, 0.0, None, "No input (centered)"),

        # Straight movements
        (0.0, -1.0, 'f', "Straight forward (max)"),
        (0.0, -0.5, 'f', "Straight forward (medium)"),
        (0.1, -1.0, 'f', "Nearly straight forward"),
        (0.24, -1.0, 'f', "Straight forward (just below 0.25)"),
        (0.0, 1.0, 'b', "Straight backward (max)"),
        (0.0, 0.5, 'b', "Straight backward (medium)"),
        (-0.1, 1.0, 'b', "Nearly straight backward"),

        # Full left/right turns (abs(a0) >= 0.25, abs(a1) <= 0.25)
        (-0.25, 0.0, 'l', "Full left (threshold)"),
        (-0.5, 0.0, 'l', "Full left (medium)"),
        (-1.0, 0.0, 'l', "Full left (max)"),
        (0.25, 0.0, 'r', "Full right (threshold)"),
        (0.5, 0.0, 'r', "Full right (medium)"),
        (1.0, 0.0, 'r', "Full right (max)"),
        (-0.8, 0.2, 'l', "Full left (slight forward)"),
        (0.8, -0.2, 'r', "Full right (slight backward)"),

        # Moderate forward turns (0.25 <= abs(a0) < 0.75, forward dominant)
        (-0.25, -1.0, 'fl_0.75', "Forward-left 75% (threshold)"),
        (-0.5, -1.0, 'fl_0.75', "Forward-left 75% (medium)"),
        (-0.7, -1.0, 'fl_0.75', "Forward-left 75% (high)"),
        (0.25, -1.0, 'fr_0.75', "Forward-right 75% (threshold)"),
        (0.5, -1.0, 'fr_0.75', "Forward-right 75% (medium)"),
        (0.7, -1.0, 'fr_0.75', "Forward-right 75% (high)"),

        # Strong forward turns (abs(a0) >= 0.75, forward dominant)
        (-0.75, -1.0, 'fl_0.5', "Forward-left 50% (threshold)"),
        (-0.9, -1.0, 'fl_0.5', "Forward-left 50% (high)"),
        (-1.0, -1.0, 'fl_0.5', "Forward-left 50% (max)"),
        (0.75, -1.0, 'fr_0.5', "Forward-right 50% (threshold)"),
        (0.9, -1.0, 'fr_0.5', "Forward-right 50% (high)"),
        (1.0, -1.0, 'fr_0.5', "Forward-right 50% (max)"),

        # Moderate backward turns (0.25 <= abs(a0) < 0.75, backward dominant)
        (-0.25, 1.0, 'bl_0.75', "Backward-left 75% (threshold)"),
        (-0.5, 1.0, 'bl_0.75', "Backward-left 75% (medium)"),
        (-0.7, 1.0, 'bl_0.75', "Backward-left 75% (high)"),
        (0.25, 1.0, 'br_0.75', "Backward-right 75% (threshold)"),
        (0.5, 1.0, 'br_0.75', "Backward-right 75% (medium)"),
        (0.7, 1.0, 'br_0.75', "Backward-right 75% (high)"),

        # Strong backward turns (abs(a0) >= 0.75, backward dominant)
        (-0.75, 1.0, 'bl_0.5', "Backward-left 50% (threshold)"),
        (-0.9, 1.0, 'bl_0.5', "Backward-left 50% (high)"),
        (-1.0, 1.0, 'bl_0.5', "Backward-left 50% (max)"),
        (0.75, 1.0, 'br_0.5', "Backward-right 50% (threshold)"),
        (0.9, 1.0, 'br_0.5', "Backward-right 50% (high)"),
        (1.0, 1.0, 'br_0.5', "Backward-right 50% (max)"),

        # Edge cases: turning dominant (abs(a0) > abs(a1))
        (-1.0, -0.5, 'l', "Left dominant over forward"),
        (1.0, -0.5, 'r', "Right dominant over forward"),
        (-1.0, 0.5, 'l', "Left dominant over backward"),
        (1.0, 0.5, 'r', "Right dominant over backward"),

        # Diagonal cases (equal magnitude)
        (-0.7, -0.7, 'fl_0.75', "Diagonal forward-left (equal)"),
        (0.7, -0.7, 'fr_0.75', "Diagonal forward-right (equal)"),
        (-0.7, 0.7, 'bl_0.75', "Diagonal backward-left (equal)"),
        (0.7, 0.7, 'br_0.75', "Diagonal backward-right (equal)"),
    ]

    # Run tests
    passed = 0
    failed = 0

    for axis0, axis1, expected, description in test_cases:
        result = get_joystick_direction(axis0, axis1, analog_mode=True)
        status = "✓" if result == expected else "✗"

        if result == expected:
            passed += 1
            print(f"{status} PASS: {description:45} | a0={axis0:5.2f}, a1={axis1:5.2f} → {result}")
        else:
            failed += 1
            print(f"{status} FAIL: {description:45} | a0={axis0:5.2f}, a1={axis1:5.2f}")
            print(f"         Expected: {expected}, Got: {result}")

    print()
    print("=" * 70)
    print(f"Results: {passed} passed, {failed} failed out of {len(test_cases)} tests")
    print("=" * 70)
    print()

    # Test binary mode as well
    print("=" * 70)
    print("Testing get_joystick_direction() - Binary Mode")
    print("=" * 70)
    print()

    binary_tests = [
        (0.0, 0.0, None, "No input"),
        (0.0, -1.0, 'f', "Forward"),
        (0.0, 1.0, 'b', "Backward"),
        (-1.0, 0.0, 'l', "Left"),
        (1.0, 0.0, 'r', "Right"),
        (-0.5, -0.5, 'f', "Diagonal forward-left (forward priority)"),
        (0.5, 1.0, 'b', "Diagonal backward-right (backward priority)"),
    ]

    binary_passed = 0
    binary_failed = 0

    for axis0, axis1, expected, description in binary_tests:
        result = get_joystick_direction(axis0, axis1, analog_mode=False)
        status = "✓" if result == expected else "✗"

        if result == expected:
            binary_passed += 1
            print(f"{status} PASS: {description:45} | a0={axis0:5.2f}, a1={axis1:5.2f} → {result}")
        else:
            binary_failed += 1
            print(f"{status} FAIL: {description:45} | a0={axis0:5.2f}, a1={axis1:5.2f}")
            print(f"         Expected: {expected}, Got: {result}")

    print()
    print("=" * 70)
    print(f"Results: {binary_passed} passed, {binary_failed} failed out of {len(binary_tests)} tests")
    print("=" * 70)

    # Summary
    total_passed = passed + binary_passed
    total_failed = failed + binary_failed
    total_tests = len(test_cases) + len(binary_tests)

    print()
    print("=" * 70)
    print(f"OVERALL: {total_passed}/{total_tests} tests passed")
    if total_failed == 0:
        print("✓ All tests passed!")
    else:
        print(f"✗ {total_failed} test(s) failed")
    print("=" * 70)


def test_live_joystick():
    """Interactive test with real joystick input."""
    pygame.init()
    pygame.joystick.init()

    # Check for joystick
    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("No joystick detected!")
        print("Exiting...")
        return

    # Initialize first joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print("=" * 70)
    print(f"Joystick detected: {joystick.get_name()}")
    print(f"Axes: {joystick.get_numaxes()}, Buttons: {joystick.get_numbuttons()}")
    print("=" * 70)
    print()
    print("Move the joystick to see detected directions")
    print("Press ESC or Ctrl+C to exit")
    print()
    print("=" * 70)

    window = pygame.display.set_mode((400, 300))
    pygame.display.set_caption("Joystick Direction Test")
    clock = pygame.time.Clock()

    deadzone = 0.1
    last_direction_binary = None
    last_direction_analog = None

    running = True
    while running:
        clock.tick(60)

        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False

        # Update joystick state
        pygame.event.pump()

        # Read axes with deadzone
        axis0 = apply_deadzone(joystick.get_axis(0), deadzone)
        axis1 = apply_deadzone(joystick.get_axis(1), deadzone)

        # Get directions
        direction_binary = get_joystick_direction(axis0, axis1, analog_mode=False)
        direction_analog = get_joystick_direction(axis0, axis1, analog_mode=True)

        # Print only when direction changes
        if direction_binary != last_direction_binary or direction_analog != last_direction_analog:
            print(f"Axis0: {axis0:6.2f} | Axis1: {axis1:6.2f} | Binary: {str(direction_binary):4} | Analog: {direction_analog}")
            last_direction_binary = direction_binary
            last_direction_analog = direction_analog

    joystick.quit()
    pygame.quit()
    print("\nTest ended.")


if __name__ == "__main__":
    # Check if user wants automated tests or live test
    if len(sys.argv) > 1 and sys.argv[1] == "--auto":
        test_joystick_direction()
    else:
        test_live_joystick()
