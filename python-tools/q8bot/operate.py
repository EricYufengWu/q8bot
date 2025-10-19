'''
Written by yufeng.wu0902@gmail.com

This is the latest control script for Q8bot (using ESPNow). Run this script 
in your laptop with an ESP32C3 connected and control the robot via keyboard.
'''

import time
import pygame
import sys
import serial.tools.list_ports
from kinematics_solver import *
from espnow import q8_espnow
from helpers import *
from trajectory_generator import (
    generate_trot_trajectories,
    generate_walk_trajectories,
    generate_bound_trajectories,
    generate_pronk_trajectories
)
from input_handler import InputHandler, detect_and_init_joystick

# User-modifiable constants
SPEED = 200
res = 0.2
y_min = 15

# Gait params dictionary. Add your own gaits here
# 'NAME': [STACKTYPE, x0, y0, xrange, yrange, yrange2, s1_count, s2_count]
gaits = {
    'TROT':      ['trot', 9.75, 43.36, 40, 20, 0, 15, 30],
    'TROT_HIGH': ['trot', 9.75, 60, 20, 10, 0, 15, 30],
    'TROT_LOW':  ['trot', 9.75, 25, 20, 10, 0, 15, 30],
    'TROT_FAST': ['trot', 9.75, 43.36, 50, 20, 0, 12, 24],
    'WALK':      ['walk',  9.75, 43.36, 30, 20, 0, 20, 140],
    'BOUND':     ['bound', 9.75, 33.36, 40, 0, 20, 50, 10],
    'PRONK':     ['pronk', 9.75, 33.36, 40, 0, 20, 60, 10]
}

# Helper Functions
def find_xiao_com_port():
    # Finds the first connected Seeed Studio XIAO ESP32C3 based on VID/PID.
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if port.vid == XIAO_VID and port.pid == XIAO_PID:
            return port.device  # Returns the COM port (e.g., "COM3" or "/dev/ttyUSB0")
    return None

def is_xiao_device(com_port):
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if port.device == com_port and port.vid == XIAO_VID and port.pid == XIAO_PID:
            return True
    return False

def show_range():
    for pos in R:
        print(pos)
        q8.move_all(pos, 1000, False)
        time.sleep(1.5)
    return

def greet(x, y):
    q8.move_all(G1, 1000, False)
    time.sleep(1.5)
    q8.move_all(G2, 1000, False)
    time.sleep(2)
    q8.move_all(G3, 500, False)
    time.sleep(1)
    for i in range (3):
        q8.move_all(G4, 200, False)
        time.sleep(0.25)
        q8.move_all(G5, 200, False)
        time.sleep(0.25)
    time.sleep(0.75)
    q8.move_all(G6, 500, False)
    time.sleep(0.7)
    move_xy(x, y, 1000)
    return

def move_xy(x, y, dur = 0, deg = True):
    q1, q2, success = leg.ik_solve(x, y, deg, 1)
    q8.move_mirror([q1, q2], dur)
    return success

def generate_trajectories_for_gait(leg, gait_name):
    """
    Generate complete trajectory set for a given gait based on its stacktype.

    Args:
        leg: Kinematics solver instance
        gait_name: Name of the gait (e.g., 'TROT', 'TROT_HIGH')

    Returns:
        Dictionary of trajectories for all movement types, or None if stacktype not supported
    """
    gait_params = gaits[gait_name]
    stacktype = gait_params[0]

    if stacktype == 'trot':
        return generate_trot_trajectories(leg, gait_params)
    elif stacktype == 'walk':
        return generate_walk_trajectories(leg, gait_params)
    elif stacktype == 'bound':
        return generate_bound_trajectories(leg, gait_params)
    elif stacktype == 'pronk':
        return generate_pronk_trajectories(leg, gait_params)
    else:
        print(f"Unsupported stacktype: {stacktype}")
        return None

def movement_start(dir, move_type = 'TROT'):
    """
    Start movement by looking up pre-calculated trajectory.
    Includes fallback logic for gaits with limited movement types.

    Args:
        dir: Direction string (e.g., 'f', 'b', 'l', 'r', 'fl_0.75', 'fl_0.5', etc.)
        move_type: Gait type name (e.g., 'TROT')

    Returns:
        Tuple of (trajectory_list, empty_list) or None if movement not found
    """
    global ongoing, current_trajectories

    # Check if we have trajectories for this gait
    if move_type not in current_trajectories:
        print(f"No trajectories loaded for {move_type}")
        return None, []

    gait_trajectories = current_trajectories[move_type]

    # Try to find exact match first
    if dir in gait_trajectories:
        ongoing = True
        return gait_trajectories[dir], []

    # Fallback logic for gaits with limited movement types
    # Try to find closest available movement
    fallback_map = {
        # Map complex turns to simpler turns if not available
        'fl_0.5': ['fl_0.75', 'fl', 'f'],  # Try gentler turn, then basic, then straight
        'fl_0.75': ['fl', 'f'],
        'fr_0.5': ['fr_0.75', 'fr', 'f'],
        'fr_0.75': ['fr', 'f'],
        'bl_0.5': ['bl_0.75', 'bl', 'b'],
        'bl_0.75': ['bl', 'b'],
        'br_0.5': ['br_0.75', 'br', 'b'],
        'br_0.75': ['br', 'b'],
        'fl': ['f'],  # Basic turns fall back to straight
        'fr': ['f'],
        'bl': ['b'],
        'br': ['b'],
    }

    # Try fallbacks
    if dir in fallback_map:
        for fallback in fallback_map[dir]:
            if fallback in gait_trajectories:
                print(f"Movement '{dir}' not available for {move_type}, using '{fallback}' instead")
                ongoing = True
                return gait_trajectories[fallback], []

    # No suitable trajectory found
    print(f"Movement type '{dir}' not available for {move_type} (no fallback found)")
    return None, []

# Main code
# Seeed Studio XIAO devices have the following VID and PID
XIAO_VID = 0x303A
XIAO_PID = 0x1001

# Flags for main loop
movement = False
ongoing = False
exit = False
record = False
request = "none"
current_direction = None  # Track current movement direction

# find a serial port and connect
if len(sys.argv) > 1:
    # User provided a COM port
    com_port = sys.argv[1]
    if not is_xiao_device(com_port):
        print(f"Error: {com_port} is not an ESP32C3 or does not exist.")
        sys.exit(1)
else:
    # Auto-detect COM port
    com_port = find_xiao_com_port()
    if com_port is None:
        print("No ESP32C3 controller device found.")
        sys.exit(1)

# Start pygame instance
pygame.init()
window = pygame.display.set_mode((300, 300))
clock = pygame.time.Clock()

# Detect and initialize input device (joystick or keyboard)
use_joystick, joystick, joystick_mapping = detect_and_init_joystick()
input_handler = InputHandler(use_joystick, joystick, joystick_mapping)

leg = k_solver(19.5, 25, 40, 25, 40)
q8 = q8_espnow(com_port)
q8.enable_torque()

# Starting location of leg end effector in x and y
gait = ['TROT', 'TROT_HIGH', 'TROT_LOW', 'TROT_FAST', 'WALK', 'BOUND', 'PRONK']
step_size = 20
pos_x = leg.d/2
pos_y = round((leg.l1 + leg.l2) * 0.667, 2)
move_xy(pos_x, pos_y, 1000)

# Pre-calculate trajectories for default gait (following flowchart)
print(f"Calculating trajectories for default gait: {gait[0]}")
current_trajectories = {}
default_gait_trajectories = generate_trajectories_for_gait(leg, gait[0])
if default_gait_trajectories:
    current_trajectories[gait[0]] = default_gait_trajectories
    print(f"Loaded {len(default_gait_trajectories)} movement types for {gait[0]}")
else:
    print(f"Failed to generate trajectories for {gait[0]}")
    sys.exit(1)

time.sleep(2)

while True:
    clock.tick(SPEED)
    pygame.event.get()

    if movement:
        # Get requested direction from input handler
        requested_direction = input_handler.get_movement_direction()

        if requested_direction:
            # Check if direction changed mid-movement
            if ongoing and requested_direction != current_direction:
                # Switch to new trajectory
                ongoing = False
                print(f"Switching from {current_direction} to {requested_direction}")

            # Load trajectory if not ongoing or direction changed
            if not ongoing:
                move_list, y_list = movement_start(requested_direction, gait[0])
                if move_list is None:  # moveFound = False
                    movement = False
                    current_direction = None
                    continue
                current_direction = requested_direction

            # Execute current trajectory
            pos, move_list = movement_tick(move_list)
            q8.move_all(pos, 0, record)
        else:
            # No movement input - transition to idle
            move_xy(pos_x, pos_y, 0)
            q8.finish_recording()
            record = False
            ongoing = False
            movement = False
            current_direction = None
    else:
        # Check for movement input
        if input_handler.is_movement_input():
            movement = True
        # Check action inputs using generalized interface
        elif input_handler.is_action_pressed('reset'):
            move_xy(pos_x, pos_y, 500)
        elif input_handler.is_action_pressed('jump'):
            print("Jump")
            q8.send_jump()
            time.sleep(5)
            move_xy(leg.d/2, (leg.l1 + leg.l2) * 0.667, 500)
            move_xy(leg.d/2, (leg.l1 + leg.l2) * 0.667, 500)
        elif input_handler.is_action_pressed('switch_gait'):
            gait.append(gait.pop(0))
            print(f"Switching to gait: {gait[0]}")

            # Completely scrap old trajectories and calculate new ones
            print(f"Calculating new trajectories for {gait[0]}...")
            current_trajectories = {}  # Clear all previous trajectories
            new_trajectories = generate_trajectories_for_gait(leg, gait[0])
            if new_trajectories:
                current_trajectories[gait[0]] = new_trajectories
                print(f"Loaded {len(new_trajectories)} movement types")
            else:
                print(f"Failed to generate trajectories for {gait[0]}")
                gait.insert(0, gait.pop())  # Revert gait change
                time.sleep(0.2)
                continue

            # Update position to match new gait
            pos_x, pos_y = gaits[gait[0]][1], gaits[gait[0]][2]
            move_xy(pos_x, pos_y, 500)
            print(f"Changed gait to: {gait[0]}")
            time.sleep(0.2)
        elif input_handler.is_action_pressed('battery'):
            q8.check_battery()
            request = "battery"
            time.sleep(0.2)
        elif input_handler.is_action_pressed('record'):
            print(f"Record next movement")
            record = True
            request = "data"
            time.sleep(0.2)
        elif input_handler.is_action_pressed('show_range'):
            print(f"Show Range")
            show_range()
            time.sleep(0.2)
        elif input_handler.is_action_pressed('greet'):
            print(f"Greet")
            greet(pos_x, pos_y)
            time.sleep(0.2)
        elif input_handler.is_action_pressed('exit'):
            break
        else:
            totalArray = []
            try:
                while q8.serialHandler.in_waiting > 0:  # Check if there is any data waiting to be read
                    raw_data = q8.serialHandler.readline().decode('utf-8').strip() # Read a line and decode it
                    raw_data = raw_data.split()
                    while raw_data and raw_data[-1] == '0':
                        raw_data.pop()
                    # print(f"Received data: {raw_data}")
                    if request == "battery":
                        print(f"Battery Level: {int(raw_data[0])}")
                    else:
                        # print(f"Received data: {raw_data}")
                        processed_data = [int(x) for x in raw_data]
                        totalArray.extend(processed_data)
                request = "none"
                if len(totalArray) > 0:
                    # print(totalArray)
                    parse_data(totalArray, move_list, y_list)
            except:
                print("Data reading failed. Continuing...")

q8.disable_torque()
if joystick:
    joystick.quit()
pygame.quit()
