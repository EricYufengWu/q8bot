'''
Written by yufeng.wu0902@gmail.com

This is the latest control script for Q8bot (using ESPNow). Run this script 
in your laptop with an ESP32C3 connected and control the robot via keyboard.
'''

import time
import math
import pygame
from kinematics_solver import *
from q8_espnow import *
from q8_helpers import *

# User-modifiable constants
PORT = 'COM6'
SPEED = 100
res = 0.2
y_min = 15

# Jumping parameters. Change this to tune jumping brhavior
JUMP_LOW = [-25, 205, -25, 205, -25, 205, -25, 205]     # 40mm leg
# JUMP_LOW = [12, 168, 12, 168, 12, 168, 12, 168]         # 35mm leg
JUMP_1 = [95, 85, 95, 85, 95, 85, 95, 85]
JUMP_2 = [95, 85, 95, 85, -25, 205, -25, 205]
JUMP_REST = [10, 170, 10, 170, 10, 170, 10, 170]            # 40mm leg
# JUMP_REST = [20, 160, 20, 160, 20, 160, 20, 160]        # 35mm leg

# Helper Functions
def jump(JUMP_HIGH, jump_time):
    # q8.move_all(JUMP_HIGH, 500, False)
    # time.sleep(0.7)
    q8.move_all(JUMP_LOW, 500, False)
    time.sleep(0.8)
    q8.move_all(JUMP_HIGH, 0, False)
    time.sleep(jump_time)
    q8.move_all(JUMP_REST, 0, False)
    time.sleep(0.5)
    return

def move_xy(x, y, dur = 0, deg = True):
    q1, q2, success = leg.ik_solve(x, y, deg, 1)
    q8.move_mirror([q1, q2], dur)
    return success

def movement_start(leg, dir, x_0, y_0, x_size, y_size, move_type = 'AMBER'):
    # start movement by generating the position list
    global ongoing
    ongoing = True
    if move_type == 'WALK': # three feet on the ground
        return generate_walk(leg, dir, x_0, y_0, x_size + 10, y_size)
    elif move_type == 'AMBER':  # this is like a smoother trot
        return generate_amber(leg, dir, x_0, y_0, x_size + 20, y_size)
    elif move_type == 'GALLOP': # two front and two back
        return generate_gallop(leg, dir, x_0, y_0 - 10, x_size + 10, y_size)
    elif move_type == 'PRONK': # four leg jump forward
        return generate_pronk(leg, dir, x_0, y_0, x_size, y_size)
    else:
        return dummy_movement()

# # Main code
# def main():
movement = False
ongoing = False
exit = False
record = False
request = "none"

pygame.init()
window = pygame.display.set_mode((300, 300))
clock = pygame.time.Clock()

leg = k_solver(19.5, 25, 40, 25, 40) # 40mm leg
q8 = q8_espnow(PORT)
q8.enable_torque()

# Starting location of leg end effector in x and y
gait = ['WALK', 'AMBER', 'GALLOP', 'PRONK']
step_size = 20
pos_x = leg.d/2
pos_y = (leg.l1 + leg.l2) * 0.667
move_xy(pos_x, pos_y, 1000)
time.sleep(2)

while True:
    clock.tick(SPEED)
    pygame.event.get()
    keys = pygame.key.get_pressed()

    if movement:
        if keys[pygame.K_w]:
            if not ongoing:
                move_list = movement_start(leg, 'f', pos_x, pos_y, step_size, step_size, gait[0])
            pos, move_list = movement_tick(move_list)
            q8.move_all(pos, 0, record)
        elif keys[pygame.K_s]:
            if not ongoing:
                move_list = movement_start(leg, 'b', pos_x, pos_y, step_size, step_size, gait[0])
            pos, move_list = movement_tick(move_list)
            q8.move_all(pos, 0, record)
        elif keys[pygame.K_a]:
            if not ongoing:
                move_list = movement_start(leg, 'l', pos_x, pos_y, step_size, step_size, gait[0])
            pos, move_list = movement_tick(move_list)
            q8.move_all(pos, 0, record)
        elif keys[pygame.K_d]:
            if not ongoing:
                move_list = movement_start(leg, 'r', pos_x, pos_y, step_size, step_size, gait[0])
            pos, move_list = movement_tick(move_list)
            q8.move_all(pos, 0, record)
        elif keys[pygame.K_q]:
            if not ongoing:
                move_list = movement_start(leg, 'fl', pos_x, pos_y, step_size, step_size, gait[0])
            pos, move_list = movement_tick(move_list)
            q8.move_all(pos, 0, record)
        elif keys[pygame.K_e]:
            if not ongoing:
                move_list = movement_start(leg, 'fr', pos_x, pos_y, step_size, step_size, gait[0])
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
        elif keys[pygame.K_UP] or keys[pygame.K_DOWN]:        # standing height
            temp_y = pos_y + (keys[pygame.K_UP] - keys[pygame.K_DOWN]) * res
            q1, q2, success = leg.ik_solve(pos_x, temp_y, True, 1)
            if success and temp_y > y_min:
                q8.move_mirror([q1,q2], 0)
                pos_y = round(temp_y, 1)
                print("Changed y height to: ", pos_y)
        elif keys[pygame.K_EQUALS] or keys[pygame.K_MINUS]:   # step size
            temp_s = step_size+(keys[pygame.K_EQUALS]-keys[pygame.K_MINUS])*res
            if temp_s > 10 and temp_s < 30:
                step_size = round(temp_s, 1)
                print("Changed step size to:", step_size)
        elif keys[pygame.K_r]:                                # reset step
            pos_x = leg.d/2
            pos_y = (leg.l1 + leg.l2) * 0.667
            move_xy(pos_x, pos_y, 500)
            step_size = 20
        elif keys[pygame.K_j]:
            print("Jump")
            # q8.send_jump()
            jump(JUMP_1, 0.1)
            time.sleep(1)
            move_xy(pos_x, pos_y, 500) # This is super weird fix this
            move_xy(pos_x, pos_y, 500)
            time.sleep(1)
        elif keys[pygame.K_g]:
            gait.append(gait.pop(0))
            print(f"Changed gait to: {gait[0]}")
            time.sleep(0.2)
        elif keys[pygame.K_b]:
            # print(f"Requesting battery info")
            q8.check_battery()
            request = "battery"
            time.sleep(0.2)
        elif keys[pygame.K_z]:
            print(f"Record next movement")
            record = True
            request = "data"
            time.sleep(0.2)
        elif keys[pygame.K_ESCAPE]:
            break
        else:
            totalArray = []
            while q8.serialHandler.in_waiting > 0:  # Check if there is any data waiting to be read
                raw_data = q8.serialHandler.readline().decode('utf-8').strip() # Read a line and decode it
                raw_data = raw_data.split()
                while raw_data and raw_data[-1] == '0':
                    raw_data.pop()
                if request == "battery":
                    print(f"Battery Level: {int(raw_data[0])}")
                else:
                    processed_data = [int(x) for x in raw_data]
                    totalArray.extend(processed_data)
            request = "none"
            if len(totalArray) > 0:
                print(totalArray) 

q8.disable_torque()
pygame.quit()

# if __name__ == "__main__":
#     main()