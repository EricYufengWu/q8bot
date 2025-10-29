'''
Written by yufeng.wu0902@gmail.com

This is the latest control script for Q8bot (using ESPNow). Run this script 
in your laptop with an ESP32C3 connected and control the robot via keyboard.
'''

import time
import pygame
import sys
import argparse
from kinematics_solver import k_solver
from espnow import q8_espnow
from helpers import XiaoPortFinder, Q8Logger
from gait_manager import GaitManager, GAITS
from routine_generator import show_range, greet
from input_handler import InputHandler, detect_and_init_joystick

# Q8bot leg configuration
CENTER_DIST = 19.5  # Distance between two actuators
L1 = 25             # Upper leg length
L2 = 40             # Lower leg length

# Pygame config
SPEED = 200
res = 0.2

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
window = pygame.display.set_mode((1280, 720))
clock = pygame.time.Clock()

# Set up pygame surface for logger
Q8Logger.set_pygame_surface(window)

# Detect and initialize input device (joystick or keyboard)
use_joystick, joystick, joystick_mapping = detect_and_init_joystick()
input_handler = InputHandler(use_joystick, joystick, joystick_mapping)

# Load appropriate instruction image based on input device
import os

def get_resource_path(relative_path):
    """Get absolute path to resource, works for dev and for PyInstaller"""
    try:
        # PyInstaller creates a temp folder and stores path in _MEIPASS
        base_path = sys._MEIPASS
    except Exception:
        # Running in development mode
        base_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(base_path, relative_path)

if use_joystick:
    instruction_image_path = get_resource_path(os.path.join("docs", "Instruction_Joystick.jpg"))
else:
    instruction_image_path = get_resource_path(os.path.join("docs", "Instruction_Default.jpg"))

try:
    instruction_image = pygame.image.load(instruction_image_path)
    # Scale image to fit the available space (1280x570)
    instruction_image = pygame.transform.scale(instruction_image, (1280, 570))
    log.info(f"Loaded instruction image: {os.path.basename(instruction_image_path)}")
except Exception as e:
    log.warning(f"Failed to load instruction image: {e}")
    instruction_image = None

# Initialize kinamatics solver and Q8bot ESPNow instance
leg = k_solver(CENTER_DIST, L1, L2, L1, L2)
q8 = q8_espnow(com_port)
q8.enable_torque()

# Initialize GaitManager
gait_names = list(GAITS.keys())
gait_manager = GaitManager(leg, GAITS)

# Starting location of leg end effector in x and y
pos_x = leg.d/2
pos_y = round((leg.l1 + leg.l2) * 0.667, 2)
move_xy(pos_x, pos_y, 1000)

# Pre-calculate trajectories for default gait
if not gait_manager.load_gait(gait_names[0]):
    log.error(f"Failed to load default gait: {gait_names[0]}")
    sys.exit(1)

time.sleep(2)

while True:
    clock.tick(SPEED)
    pygame.event.get()

    # Clear screen and render logger messages
    window.fill((0, 0, 0))  # Black background

    # Draw instruction image below logger (if loaded)
    if instruction_image is not None:
        window.blit(instruction_image, (0, 150))  # Position at y=150 (below logger)

    Q8Logger.render_pygame_messages()  # Draw logger on top
    pygame.display.flip()

    if movement:
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
        elif input_handler.is_action_pressed('jump'):
            log.info("Jump")
            q8.send_jump()
            time.sleep(5)
            move_xy(leg.d/2, (leg.l1 + leg.l2) * 0.667, 500)
            move_xy(leg.d/2, (leg.l1 + leg.l2) * 0.667, 500)
        elif input_handler.is_action_pressed('switch_gait'):
            # Cycle to next gait
            gait_names.append(gait_names.pop(0))
            new_gait = gait_names[0]

            # Load new gait
            if gait_manager.load_gait(new_gait):
                # Update position to match new gait
                pos_x, pos_y = GAITS[new_gait][1], GAITS[new_gait][2]
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
        else:
            try:
                while q8.serialHandler.in_waiting > 0:  # Check if there is any data waiting to be read
                    raw_data = q8.serialHandler.readline().decode('utf-8').strip() # Read a line and decode it
                    raw_data = raw_data.split()
                    while raw_data and raw_data[-1] == '0':
                        raw_data.pop()
                    # print(f"Received data: {raw_data}")
                    if request == "battery":
                        log.info(f"Battery: {int(raw_data[0])}%")
                request = "none"
            except:
                log.debug("Data reading failed. Continuing...")

q8.disable_torque()
if joystick:
    joystick.quit()
pygame.quit()
