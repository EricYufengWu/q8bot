# This is the latest control script for Q8bot (using ESPNow)

import time
import math
import pygame
from kinematics_solver import *
from q8_espnow import *

# User-modifiable constants
SPEED = 100
res = 0.5
y_min = 15
# PORT = 'COM6'    #ESP1
PORT = 'COM4'     #ESP2

# Jumping parameters. Change this to tune jumping brhavior
# JUMP_LOW = [-25, 205, -25, 205, -25, 205, -25, 205]   # 40mm leg
JUMP_LOW = [12, 168, 12, 168, 12, 168, 12, 168]
JUMP_1 = [95, 85, 95, 85, 95, 85, 95, 85]
JUMP_2 = [95, 85, 95, 85, -25, 205, -25, 205]
# JUMP_REST = [0, 180, 0, 180, 0, 180, 0, 180]   # 40mm leg
JUMP_REST = [20, 160, 20, 160, 20, 160, 20, 160]


# Helper Functions
def jump(JUMP_HIGH, jump_time):
    # q8.move_all(JUMP_HIGH, 500)
    # time.sleep(0.7)
    q8.move_all(JUMP_LOW, 500)
    time.sleep(0.8)
    q8.move_all(JUMP_HIGH, 0)
    time.sleep(jump_time)
    q8.move_all(JUMP_REST, 0)
    time.sleep(0.5)
    return

def generate_diag_amber(leg, dir, x0 = 9.75, y0 = 40, yrange = 20.0, 
                        xrange = 20.0, lift_count = 10, len_factor = 3):
    # Generate two sets of single-leg position lists (forward + backward). 
    # Mix and match to create an aggregated list for 4 legs.
    move_p, move_n = [], []
    x_p, x_n = x0 - xrange / 2, x0 + xrange / 2
    x_lift_step = xrange / lift_count
    x_down_step = xrange / lift_count / (len_factor - 1)

    # validate_movement()
    # a1, a2, success = leg.ik_solve(x0 + , y, deg, 1)

    for i in range(lift_count * len_factor):
        if i < lift_count:
            x_p += x_lift_step * min(i, 1)
            x_n -= x_lift_step * min(i, 1)
            y = y0 - math.sin((i+1)*(math.pi/(lift_count+1))) * yrange
        else:
            x_p -= x_down_step
            x_n += x_down_step
            y = y0
        q1_p, q2_p, success = leg.ik_solve(x_p, y, True, 1)
        q1_n, q2_n, success = leg.ik_solve(x_n, y, True, 1)
        move_p.append([q1_p, q2_p])
        move_n.append([q1_n, q2_n])

    split = int(lift_count * len_factor/2)
    move_p2 = move_p[split:] + move_p[:split]
    move_n2 = move_n[split:] + move_n[:split]
    if dir == 'f':
        return append_pos_list(move_p, move_p2, move_p2, move_p)
    elif dir == 'r':
        return append_pos_list(move_p, move_n2, move_p2, move_n)
    elif dir == 'l':
        return append_pos_list(move_n, move_p2, move_n2, move_p)
    else: # all other conditions default to "back"
        return append_pos_list(move_n, move_n2, move_n2, move_n)
    
def validate_movement():
    return

def append_pos_list(list_1, list_2, list_3, list_4):
    # Append values to overall movement list in specific order. 
    #    Robot Front
    # list 1      list 2
    # list 3      list 4
    append_list = []
    for i in range(len(list_1)):
        append_list.append([list_1[i][0], list_1[i][1], 
                           list_2[i][0], list_2[i][1], 
                           list_3[i][0], list_3[i][1], 
                           list_4[i][0], list_4[i][1]])
    return append_list

def movement_start(leg, dir, x_0, y_0, move_type = 'AMBER'):
    # start movement by generating the position list
    global ongoing
    ongoing = True
    if move_type == 'WALK':
        return dummy_movement()
    elif move_type == 'AMBER':
        return generate_diag_amber(leg, dir, x_0, y_0)
    elif move_type == 'TROT':
        return dummy_movement()
    elif move_type == 'GALLOP':
        return dummy_movement()
    else:
        return dummy_movement()

def dummy_movement():
    # Movement placeholder for the types I have yet to write atual code for :/
    return[[90 for i in range(8)] for j in range(10)]

def movement_tick(move_list):
    # For each step in a movement, cycle through pre-calculated list.
    pos = move_list[0]
    new_list = move_list[1:] + [move_list[0]]
    return pos, new_list

def move_xy(x, y, dur = 0, deg = True):
    q1, q2, success = leg.ik_solve(x, y, deg, 1)
    q8.move_mirror([q1, q2], dur)
    return success

# # Main code
# def main():
movement = False
ongoing = False
exit = False

pygame.init()
window = pygame.display.set_mode((300, 300))
clock = pygame.time.Clock()

leg = k_solver(19.5, 25, 35, 25, 35)
q8 = q8_espnow(PORT)
q8.enable_torque()

# Starting location of leg end effector in x and y
pos_x = leg.d/2
pos_y = (leg.l1 + leg.l2) * 0.667
move_xy(pos_x, pos_y, 1000)
# q1_idle, q2_idle, success = leg.ik_solve(pos_x, pos_y, True, 1)
# q8.move_mirror([q1_idle, q2_idle], 1000)
time.sleep(2)

while True:
    clock.tick(SPEED)
    pygame.event.get()
    keys = pygame.key.get_pressed()

    if movement:
        if keys[pygame.K_w]:
            if not ongoing:
                move_list = movement_start(leg, 'f', pos_x, pos_y)
            pos, move_list = movement_tick(move_list)
            q8.move_all(pos, 0)
        elif keys[pygame.K_s]:
            if not ongoing:
                move_list = movement_start(leg, 'b', pos_x, pos_y)
            pos, move_list = movement_tick(move_list)
            q8.move_all(pos, 0)
        elif keys[pygame.K_a]:
            if not ongoing:
                move_list = movement_start(leg, 'l', pos_x, pos_y)
            pos, move_list = movement_tick(move_list)
            q8.move_all(pos, 0)
        elif keys[pygame.K_d]:
            if not ongoing:
                move_list = movement_start(leg, 'r', pos_x, pos_y)
            pos, move_list = movement_tick(move_list)
            q8.move_all(pos, 0)
        else:
            move_xy(pos_x, pos_y, 0)
            ongoing = False
            movement = False
    else:
        if (keys[pygame.K_w] or keys[pygame.K_a] or 
            keys[pygame.K_s] or keys[pygame.K_d]):
            movement = True
        elif keys[pygame.K_UP] or keys[pygame.K_DOWN]:
            # temp_x = pos_x + (keys[pygame.K_LEFT] - keys[pygame.K_RIGHT]) * res
            temp_y = pos_y + (keys[pygame.K_UP] - keys[pygame.K_DOWN]) * res
            q1, q2, success = leg.ik_solve(pos_x, temp_y, True, 1)
            if success and temp_y > y_min:
                q8.move_mirror([q1,q2], 0)
                pos_y = temp_y
        elif keys[pygame.K_b]:
            voltage = q8.check_voltage()
            q8.check_battery()
            print("Battery Voltage: %.1f" % (voltage))
            time.sleep(0.2)
        elif keys[pygame.K_j]:
            print("Jump")
            # q8.send_jump()
            jump(JUMP_1, 0.1)
            time.sleep(1)
            # q8.move_mirror([q1_idle, q2_idle], 500)
            # q8.move_mirror([q1_idle, q2_idle], 500)
            time.sleep(1)
        elif keys[pygame.K_ESCAPE]:
            break


q8.disable_torque()
pygame.quit()

# if __name__ == "__main__":
#     main()