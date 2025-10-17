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

# User-modifiable constants
PORT = 'COM14'
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
    # 'WALK':       ['walk',  9.75, 43.36, 30, 20, 0, 20, 140],
    # 'BOUND':      ['bound', 9.75, 33.36, 40, 0, 20, 50, 10],
    # 'PRONK':      ['pronk', 9.75, 33.36, 40, 0, 20, 60, 10]
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

def movement_start(leg, dir, move_type = 'TROT'):
    # start movement by generating the position list
    global ongoing
    ongoing = True
    return generate_gait(leg, dir, gaits[move_type])

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

leg = k_solver(19.5, 25, 40, 25, 40)
q8 = q8_espnow(com_port)
q8.enable_torque()

# Starting location of leg end effector in x and y
gait = ['TROT', 'TROT_HIGH', 'TROT_LOW', 'TROT_FAST'] #, 'WALK', 'BOUND', 'PRONK']
step_size = 20
pos_x = leg.d/2
pos_y = round((leg.l1 + leg.l2) * 0.667, 2)
move_xy(pos_x, pos_y, 1000)
time.sleep(2)

while True:
    clock.tick(SPEED)
    pygame.event.get()
    keys = pygame.key.get_pressed()

    if movement:
        if keys[pygame.K_w]:
            if not ongoing:
                move_list, y_list = movement_start(leg, 'f', gait[0])
            pos, move_list = movement_tick(move_list)
            q8.move_all(pos, 0, record)
        elif keys[pygame.K_s]:
            if not ongoing:
                move_list, y_list = movement_start(leg, 'b', gait[0])
            pos, move_list = movement_tick(move_list)
            q8.move_all(pos, 0, record)
        elif keys[pygame.K_a]:
            if not ongoing:
                move_list, y_list = movement_start(leg, 'l', gait[0])
            pos, move_list = movement_tick(move_list)
            q8.move_all(pos, 0, record)
        elif keys[pygame.K_d]:
            if not ongoing:
                move_list, y_list = movement_start(leg, 'r', gait[0])
            pos, move_list = movement_tick(move_list)
            q8.move_all(pos, 0, record)
        elif keys[pygame.K_q]:
            if not ongoing:
                move_list, y_list = movement_start(leg, 'fl', gait[0])
            pos, move_list = movement_tick(move_list)
            q8.move_all(pos, 0, record)
        elif keys[pygame.K_e]:
            if not ongoing:
                move_list, y_list = movement_start(leg, 'fr', gait[0])
            pos, move_list = movement_tick(move_list)
            q8.move_all(pos, 0, record)
        else:
            move_xy(pos_x, pos_y, 0)
            q8.finish_recording()
            record = False
            ongoing = False
            movement = False
    else:
        if (keys[pygame.K_w] or keys[pygame.K_a] or keys[pygame.K_s] or 
            keys[pygame.K_d] or keys[pygame.K_q] or keys[pygame.K_e] ):
            movement = True
        elif keys[pygame.K_r]:                                # reset step
            move_xy(pos_x, pos_y, 500)
        elif keys[pygame.K_j]:
            print("Jump")
            q8.send_jump()
            time.sleep(5)
            move_xy(leg.d/2, (leg.l1 + leg.l2) * 0.667, 500)
            move_xy(leg.d/2, (leg.l1 + leg.l2) * 0.667, 500)
        elif keys[pygame.K_g]:
            gait.append(gait.pop(0))
            print(f"Changed gait to: {gait[0]}")
            pos_x, pos_y = gaits[gait[0]][1], gaits[gait[0]][2]
            move_xy(pos_x, pos_y, 500)
            time.sleep(0.2)
        elif keys[pygame.K_b]:
            q8.check_battery()
            request = "battery"
            time.sleep(0.2)
        elif keys[pygame.K_z]:
            print(f"Record next movement")
            record = True
            request = "data"
            time.sleep(0.2)
        elif keys[pygame.K_c]:
            print(f"Show Range")
            show_range()
            time.sleep(0.2)
        elif keys[pygame.K_h]:
            print(f"Greet")
            greet(pos_x, pos_y)
            time.sleep(0.2)
        elif keys[pygame.K_ESCAPE]:
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
pygame.quit()
