import time
import math
import pygame
from kinematics_solver import *
from q8_dynamixel import *

speed = 100
movement = False
ongoing = False
exit = False

def pos_list(q1, q2):
    cmd_pos = []
    for i in range(4):
        cmd_pos.append(q1)
        cmd_pos.append(q2)
    return cmd_pos

def generate_movement(leg, dir, y_idle = 40, x_idle = 9.75, yrange = 20.0, 
                      xrange = 20.0, lift_count = 10, len_factor = 3):
    move_1 = []
    x = x_idle + dir * xrange / 2
    x_lift_step = xrange / lift_count
    x_down_step = xrange / lift_count / (len_factor - 1)

    for i in range(lift_count * len_factor):
        if i < lift_count:
            x += -dir * x_lift_step * min(i, 1)
            y = y_idle - math.sin((i+1)*(math.pi/(lift_count+1))) * yrange
        else:
            x += dir * x_down_step
            y = y_idle
        q1, q2, sucess = leg.ik_solve(x, y)
        move_1.append([q1, q2])

    split = int(len(move_1)/2)
    move_2 = move_1[split:] + move_1[:split]
    diag_amble_list = []
    for i in range(lift_count * len_factor):
        diag_amble_list.append([move_1[i][0], move_1[i][1], 
                                move_2[i][0], move_2[i][1], 
                                move_2[i][0], move_2[i][1], 
                                move_1[i][0], move_1[i][1]])
    return diag_amble_list

def movement_start(leg, dir):
    global ongoing
    ongoing = True
    return generate_movement(leg, dir)

def movement_tick(move_list):
    pos = move_list[0]
    new_list = move_list[1:] + [move_list[0]]
    return pos, new_list

def transition_to_idle():
    # Move all joint to idle position immediately
    global ongoing
    ongoing = False
    return

pygame.init()
window = pygame.display.set_mode((300, 300))
clock = pygame.time.Clock()

leg = k_solver()
q8 = q8_dynamixel('COM3')
q8.enable_torque()

pos_x = leg.d/2
pos_y = (leg.l1 + leg.l2) * 0.667
q1_idle, q2_idle, success = leg.ik_solve(pos_x, pos_y)
q8.move_all(pos_list(q1_idle, q2_idle), 1000)
time.sleep(2)

while True:
    clock.tick(speed)
    pygame.event.get()
    keys = pygame.key.get_pressed()

    if movement:
        if keys[pygame.K_w]:
            if not ongoing:
                move_list = movement_start(leg, 1)
            pos, move_list = movement_tick(move_list)
            # print(pos)
            q8.move_all(pos, 0)
        elif keys[pygame.K_s]:
            if not ongoing:
                move_list = movement_start(leg, -1)
            pos, move_list = movement_tick(move_list)
            # print(pos)
            q8.move_all(pos, 0)
        else:
            transition_to_idle()
            q8.move_all(pos_list(q1_idle, q2_idle), 0)
            movement = False
    else:
        # print("Idle") 
        if (keys[pygame.K_w] or keys[pygame.K_a] or 
            keys[pygame.K_s] or keys[pygame.K_d]):
            movement = True
        elif keys[pygame.K_b]:
            print("Battery")
            time.sleep(0.5)
        elif keys[pygame.K_j]:
            print("Jump")
            time.sleep(0.5)
        elif keys[pygame.K_ESCAPE]:
            break

q8.disable_torque()
pygame.quit()
exit()