'''
Written by yufeng.wu0902@gmail.com

This is the latest control script for Q8bot (using ESPNow). Run this script
in your laptop with an ESP32C3 connected and control the robot via keyboard.

At startup the fitted leg design is detected before anything moves (see
leg_detection.py), unless it is given with --legs.
'''

import time
import pygame
import sys
import argparse
import os
from espnow import q8_espnow, parse_values, decode_reply
from helpers import XiaoPortFinder, Q8Logger
from gait_manager import GaitManager
from routine_generator import show_range, greet
from input_handler import InputHandler, detect_and_init_joystick
from sitting_pose import SittingPose
from rearing_pose import RearingPose
from control_panel import ControlPanel, WINDOW_SIZE
import eth_theme as eth
from leg_calibration import LegCalibration, CONFIRM, FAILED, HOLD, PICK
from leg_designs import LEG_DESIGNS, design_for
from leg_detection import LegDetector, head_limits, save_last_design
from head_control import HeadControl
from squro_gait import SquroTrot
from stand_up import StandUp, FULL_PWM
from startup_flow import StartupFlow

# Pygame config
SPEED = 200
res = 0.2

# Actions that drive a gait or a single-model pose. They stay off for leg
# designs whose hind legs follow their own model (see LegDesign.supports_gaits).
GAIT_ACTIONS = ('reset', 'recover', 'jump', 'switch_gait', 'record', 'show_range', 'greet')

# Helper Functions
def move_xy(x, y, dur = 0, deg = True):
    """Move robot legs to specific x,y position."""
    q1, q2, success = leg.ik_solve(x, y, deg, 1)
    q8.move_mirror([q1, q2], dur)
    return success

# Parse command-line arguments
parser = argparse.ArgumentParser(description='Q8bot control script')
parser.add_argument('com_port', nargs='?', help='COM port for ESP32C3 (optional, auto-detect if not provided)')
parser.add_argument('--debug', action='store_true', help='Enable debug logging')
parser.add_argument('--legs', choices=sorted(k for k, d in LEG_DESIGNS.items() if d.available),
                    help='Skip leg detection and use this fitted design')
args = parser.parse_args()

# Initialize logger
log = Q8Logger(debug=args.debug)

# Flags for main loop
movement = False
exit = False
record = False
request = "none"

# Find a serial port and connect
if args.com_port:
    # User provided a COM port
    com_port = args.com_port
    if not XiaoPortFinder.validate(com_port):
        log.error(f"{com_port} is not an ESP32C3 or does not exist.")
        sys.exit(1)
else:
    # Auto-detect COM port
    com_port = XiaoPortFinder.find()
    if com_port is None:
        log.error("No ESP32C3 controller device found.")
        sys.exit(1)

# Start pygame instance
pygame.init()
window = pygame.display.set_mode(WINDOW_SIZE)
pygame.display.set_caption('Q8bot - Gaits and poses')
clock = pygame.time.Clock()

# Set up pygame surface for logger
Q8Logger.set_pygame_surface(window)

# Detect and initialize input device (joystick or keyboard)
use_joystick, joystick, joystick_mapping = detect_and_init_joystick()
input_handler = InputHandler(use_joystick, joystick, joystick_mapping)

# Resolve images for both source runs and PyInstaller executables.
def get_resource_path(relative_path):
    """Get absolute path to resource, works for dev and for PyInstaller"""
    try:
        # PyInstaller creates a temp folder and stores path in _MEIPASS
        base_path = sys._MEIPASS
    except Exception:
        # Running in development mode
        base_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(base_path, relative_path)

control_panel = ControlPanel(get_resource_path(os.path.join('docs', 'poses')),
                             use_joystick, joystick_mapping)

q8 = q8_espnow(com_port)
calibration = LegCalibration(q8)
head = HeadControl(q8)

def load_design(design, remember=True):
    """Make ``design`` the fitted one: solver, gaits, poses, pictures. No motion."""
    global fitted_design, leg, sitting_pose, rearing_pose, gait_manager, gait_names, pos_x, pos_y, stand_up, trot
    leg = design.solver()
    gait_names = list(design.gaits.keys())
    manager = GaitManager(leg, design.gaits)
    if not manager.load_gait(gait_names[0]):
        log.error(f"{design.title}: could not generate gait {gait_names[0]}.")
        return False
    fitted_design = design
    gait_manager = manager
    sitting_pose = SittingPose(leg, q8, targets=design.sitting)
    rearing_pose = RearingPose(leg, q8, waypoints=design.rearing)
    pos_x, pos_y = design.gaits[gait_names[0]][1], design.gaits[gait_names[0]][2]
    stand_up = None if design.supports_gaits else StandUp(design, q8)
    trot = None
    if design.per_leg:
        try:
            trot = SquroTrot(design)
        except ValueError as error:
            log.error(f"Trot disabled: {error}")
    control_panel.set_design(design.key)
    calibration.fitted_key = design.key
    if remember:
        save_last_design(design.key)
    return True

def begin_operation(design):
    """Start a session with the fitted design. Only the Q8 legs move here."""
    if not load_design(design):
        sys.exit(1)
    if design.supports_gaits:
        q8.set_output_cap(FULL_PWM)    # whatever cap the servos still hold must not carry over
        q8.enable_torque()
        move_xy(pos_x, pos_y, 1000)
        time.sleep(2)
        log.info('Controls ready. G: switch gait. P: sit / stand. U: rear / lower.')
    else:
        # Torque stays off: the legs are released until the user stands it up.
        log.info(f'{design.title} legs. Set the robot down on its feet, then press T to stand up.')

def refit(design):
    """After leg calibration: the new design is held at its mounting pose."""
    if not load_design(design):
        return
    if design.supports_gaits:
        q8.set_output_cap(FULL_PWM)
        move_xy(pos_x, pos_y, 1000)
    else:
        stand_up.hold(design.mount_positions())    # restores full output itself
    log.info(f"{design.title} legs fitted.")

def advance_calibration():
    """Enter steps through the picker and then the calibration stages."""
    stage = calibration.stage
    if stage == PICK:
        if calibration.choose():
            log.info(f"Calibrating {calibration.chosen().title} legs.")
        else:
            log.info(calibration.error)
    elif stage in (CONFIRM, FAILED):
        if calibration.confirm():
            log.info("Driving to the mounting pose. Keep hands clear.")
        else:
            log.error(calibration.error)
    elif stage == HOLD:
        design = calibration.chosen()
        calibration.finish()
        refit(design)

def calibration_keys(events):
    """Calibration owns the arrow keys and Enter while it is open."""
    for event in events:
        if event.type != pygame.KEYDOWN:
            continue
        if event.key in (pygame.K_UP, pygame.K_LEFT):
            calibration.move_cursor(-1)
        elif event.key in (pygame.K_DOWN, pygame.K_RIGHT):
            calibration.move_cursor(1)
        elif event.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
            advance_calibration()

def drain():
    """Every reply waiting on the serial line: (tagged replies, other lines)."""
    replies, lines = [], []
    try:
        while q8.serialHandler.in_waiting > 0:
            raw = q8.serialHandler.readline()
            values = parse_values(raw)
            reply = decode_reply(values) if values else None
            if reply:
                replies.append(reply)
            else:
                lines.append(raw.decode('utf-8').strip().split())
    except (ValueError, UnicodeError, TypeError):
        log.debug("Data reading failed. Continuing...")
    return replies, lines

startup = None
if args.legs:
    begin_operation(design_for(args.legs))
else:
    # Placeholders until detection settles the design; nothing is armed.
    load_design(design_for('Q8'), remember=False)
    startup = StartupFlow(LegDetector(q8), log)
    startup.begin()
    log.info('Leg detection: lift the robot so the legs hang free, then press Enter.')

while True:
    clock.tick(SPEED)
    events = pygame.event.get()

    if startup is not None and startup.active:
        replies, _ = drain()
        quit_requested = startup.handle(events, replies)
        if quit_requested or any(event.type == pygame.QUIT for event in events):
            break
        startup.update()
        if not startup.active:
            detector = startup.detector
            if detector.telemetry and 19 in detector.telemetry['servos']:
                head_now = detector.telemetry['servos'][19]
                if head_now['responding']:
                    head.set_present(head_now['deg'])
            limits = head_limits(detector.head) if detector.head else None
            if limits:
                head.set_limits(limits)
                log.info(f"Head limits measured: {limits[0]:+.0f} to {limits[1]:+.0f} deg.")
            elif detector.head:
                log.warning("Head sweeps gave no usable limits; keeping the previous ones.")
            begin_operation(startup.chosen)
            continue
        window.fill(eth.BACKGROUND)
        control_panel.draw(window, gait_manager.current_gait, sitting_pose, startup=startup)
        Q8Logger.render_pygame_messages()
        pygame.display.flip()
        continue

    escape_pressed = any(event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE
                         for event in events)
    # While calibration is open Esc cancels it instead of quitting, so a leg
    # swap can be abandoned without dropping torque mid-sequence.
    if calibration.active and escape_pressed:
        calibration.cancel()
        log.info("Leg calibration cancelled.")
        escape_pressed = False
    if (any(event.type == pygame.QUIT for event in events) or escape_pressed
            or (not calibration.active and input_handler.is_action_pressed('exit'))):
        break

    calibration.update()
    if not calibration.active:
        # Head: arrows turn it inside its collision limits, Down centres it.
        if input_handler.keyboard_action_pressed_once('swap_head', events):
            head.swap_direction()
            log.info("Head left/right swapped.")
        centre_head = input_handler.keyboard_action_pressed_once('centre_head', events)
        head.update(input_handler.head_direction(), centre_head)
    if calibration.active:
        calibration_keys(events)
    elif input_handler.keyboard_action_pressed_once('change_legs', events):
        if (sitting_pose.blocks_movement() or rearing_pose.blocks_movement() or
                (stand_up is not None and stand_up.stage in ('measuring', 'capping', 'rising', 'lowering'))):
            log.info("Finish the current pose before changing legs.")
        else:
            gait_manager.stop()
            movement = False
            calibration.open_picker()

    transition_error = rearing_pose.update()
    if transition_error:
        log.error(transition_error)

    # Consume both keys every frame so holding either never repeats a toggle.
    sit_pressed = input_handler.keyboard_action_pressed_once('sit', events)
    rear_pressed = input_handler.keyboard_action_pressed_once('rear', events)
    stand_pressed = input_handler.keyboard_action_pressed_once('stand', events)
    if stand_up is not None and (sit_pressed or rear_pressed):
        log.info(f"Sitting and rearing are not enabled for {fitted_design.title} legs yet.")
    if calibration.active or stand_up is not None:
        sit_pressed = rear_pressed = False
    if rear_pressed:
        if sitting_pose.blocks_movement():
            log.info('Press P to stand before rearing.')
        else:
            try:
                toggle = rearing_pose.retry_lowering if rearing_pose.failed else rearing_pose.toggle
                if toggle(pos_x, pos_y):
                    if record:
                        q8.finish_recording()
                    record = False
                    gait_manager.stop()
                    movement = False
                    log.info('Rearing pose' if rearing_pose.active else 'Lowering to gait position')
                else:
                    log.error('Could not send rearing/lowering pose')
            except ValueError as error:
                log.error(str(error))
    elif sit_pressed and rearing_pose.blocks_movement():
        log.info('Press U to lower before sitting.')
    elif sit_pressed:
        try:
            if sitting_pose.toggle(pos_x, pos_y):
                if record:
                    q8.finish_recording()
                record = False
                gait_manager.stop()
                movement = False
                log.info("Sitting pose" if sitting_pose.active else "Returning to gait position")
            else:
                log.error("Could not send sitting/standing pose")
        except ValueError as error:
            log.error(str(error))

    # Clear screen and render logger messages
    window.fill(eth.BACKGROUND)
    if stand_up is not None:
        display_pose = stand_up
    else:
        display_pose = rearing_pose if rearing_pose.blocks_movement() else sitting_pose
    control_panel.draw(window, gait_manager.current_gait, display_pose, calibration,
                       head=head, trot=trot)
    Q8Logger.render_pygame_messages()  # Draw logger on top
    pygame.display.flip()

    if calibration.blocks_movement():
        # Torque holds the mounting pose; nothing else may command the joints.
        pass
    elif stand_up is not None:
        # Legs with a per-leg model: stand up / lie down, the slow test trot
        # while standing, and battery. Everything single-model stays off.
        moving_input = input_handler.is_movement_input()
        switch_pressed = input_handler.keyboard_action_pressed_once('switch_gait', events)
        if trot is not None and trot.walking and not moving_input:
            home, ease = trot.stop()
            q8.move_all(home, ease, False)
            stand_up.set_commanded(home)
        if stand_pressed:
            if trot is not None and trot.walking:
                log.info("Release the walking keys before lying down.")
            elif stand_up.toggle():
                log.info("Standing up..." if stand_up.stage == 'measuring' else "Lying down...")
        elif moving_input:
            if trot is None or stand_up.stage != 'standing':
                if not movement:
                    log.info("Stand up first: press T." if trot is not None
                             else f"Walking is not available for {fitted_design.title} legs.")
                movement = True
            else:
                direction = input_handler.get_movement_direction()
                if direction and trot.start_movement(direction):
                    step = trot.tick()
                    if step:
                        if q8.move_all(step[0], step[1], False):
                            stand_up.set_commanded(step[0])
                        else:
                            q8.move_all(*trot.stop(), False)
                            log.error("Could not send a trot frame; stopped.")
        elif switch_pressed and trot is not None:
            if trot.switch_gait():
                gait = trot.profile
                log.info(f"Gait: {gait.title}, {gait.stride:g} mm stride, {gait.cycle_s:.2f} s cycle.")
            else:
                log.info("Stop walking before switching gait.")
        elif input_handler.is_action_pressed('battery'):
            q8.check_battery()
            request = "battery"
            time.sleep(0.2)
        elif any(input_handler.is_action_pressed(action) for action in GAIT_ACTIONS
                 if action != 'switch_gait'):
            if not movement:
                log.info(f"That action is not enabled for {fitted_design.title} legs yet.")
            movement = True
        else:
            movement = False
        stand_up.update()
    elif sitting_pose.blocks_movement() or rearing_pose.blocks_movement():
        # Keep the static pose; battery checks and exit remain available.
        if input_handler.is_action_pressed('battery'):
            q8.check_battery()
            request = "battery"
            time.sleep(0.2)
    elif movement:
        # Get requested direction from input handler
        requested_direction = input_handler.get_movement_direction()

        if requested_direction:
            # Start or switch movement direction
            if gait_manager.start_movement(requested_direction):
                # Execute current trajectory
                pos = gait_manager.tick()
                if pos:
                    q8.move_all(pos, 0, record)
            else:
                # Failed to start movement
                movement = False
        else:
            # No movement input - transition to idle
            move_xy(pos_x, pos_y, 0)
            q8.finish_recording()
            record = False
            gait_manager.stop()
            movement = False
    else:
        # Check for movement input
        if input_handler.is_movement_input():
            movement = True
        # Check action inputs using generalized interface
        elif input_handler.is_action_pressed('reset'):
            log.info("Gait Reset")
            move_xy(pos_x, pos_y, 500)
            time.sleep(0.2)
        elif input_handler.is_action_pressed('recover'):
            log.info("Recovering servos: clearing faults and re-enabling torque")
            q8.send_recover()
            time.sleep(2)
            move_xy(pos_x, pos_y, 1000)
        elif input_handler.is_action_pressed('jump'):
            log.info("Jump")
            q8.send_jump()
            time.sleep(5)
            move_xy(pos_x, pos_y, 500)
        elif input_handler.is_action_pressed('switch_gait'):
            # Cycle to next gait
            gait_names.append(gait_names.pop(0))
            new_gait = gait_names[0]

            # Load new gait
            if gait_manager.load_gait(new_gait):
                # Update position to match new gait
                pos_x, pos_y = fitted_design.gaits[new_gait][1], fitted_design.gaits[new_gait][2]
                move_xy(pos_x, pos_y, 500)
                log.info(f"Switched to {new_gait}")
            else:
                log.error(f"Failed to load gait: {new_gait}")
                gait_names.insert(0, gait_names.pop())  # Revert gait change
            time.sleep(0.2)
        elif input_handler.is_action_pressed('battery'):
            q8.check_battery()
            request = "battery"
            time.sleep(0.2)
        elif input_handler.is_action_pressed('record'):
            log.debug("Record next movement")
            record = True
            request = "data"
            time.sleep(0.2)
        elif input_handler.is_action_pressed('show_range'):
            log.info("Show Range")
            show_range(q8)
            time.sleep(0.2)
        elif input_handler.is_action_pressed('greet'):
            log.info("Greet")
            greet(q8)
            move_xy(pos_x, pos_y, 1000)  # Return to rest position
            time.sleep(0.2)
        elif input_handler.is_action_pressed('exit'):
            break

    # Drain replies while sitting as well as while operating a gait.
    replies, lines = drain()
    for reply in replies:
        calibration.feed(reply)
        if stand_up is not None:
            stand_up.feed(reply)
        elif reply['kind'] == 'cap' and reply['confirmed'] != 0xFF:
            log.error("Not every leg servo confirmed full output; press X to recover the servos.")
    for line in calibration.reports() + (stand_up.reports() if stand_up is not None else []):
        log.error(line)
    for raw_data in lines:
        if request == "battery" and raw_data and raw_data[0].isdigit():
            log.info(f"Battery: {int(raw_data[0])}%")
            request = "none"

if startup is not None and startup.stage == 'detecting':
    q8.abort_probe()
q8.disable_torque()
if joystick:
    joystick.quit()
pygame.quit()
