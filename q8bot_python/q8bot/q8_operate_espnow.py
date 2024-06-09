import time
import math
import pygame
from kinematics_solver import *
from q8_espnow import *

# User-modifiable constants
SPEED = 100

# Jumping parameters. Change this to tune jumping brhavior
JUMP_LOW = [11, 169, 11, 169, 11, 169, 11, 169]
JUMP_1 = [95, 85, 95, 85, 95, 85, 95, 85]
JUMP_2 = [90, 90, 90, 90, 90, 90, 90, 90]
JUMP_3 = [85, 95, 85, 95, 85, 95, 85, 95]
JUMP_4 = [80, 100, 80, 100, 80, 100, 80, 100]
JUMP_REST = [20, 160, 20, 160, 20, 160, 20, 160]

# Helper Functions
def jump(JUMP_HIGH, jump_time):
    q8.move_all(JUMP_LOW, 500)
    time.sleep(0.7)
    q8.move_all(JUMP_HIGH, 0)
    time.sleep(jump_time)
    q8.move_all(JUMP_REST, 0)
    time.sleep(0.5)
    return

def generate_diag_amber(leg, dir, y0 = 40, x0 = 9.75, yrange = 20.0, 
                        xrange = 20.0, lift_count = 10, len_factor = 3):
    # Generate two sets of single-leg position lists (forward + backward). 
    # Mix and match to create an aggregated list for 4 legs.
    move_p, move_n = [], []
    x_p, x_n = x0 - xrange / 2, x0 + xrange / 2
    x_lift_step = xrange / lift_count
    x_down_step = xrange / lift_count / (len_factor - 1)

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

def movement_start(leg, dir, turn = False, move_type = 'AMBER'):
    # start movement by generating the position list
    global ongoing
    ongoing = True
    if move_type == 'WALK':
        return dummy_movement()
    elif move_type == 'AMBER':
        return generate_diag_amber(leg, dir)
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

# # Main code
# def main():
movement = False
ongoing = False
exit = False

pygame.init()
window = pygame.display.set_mode((300, 300))
clock = pygame.time.Clock()

leg = k_solver()
# q8 = q8_espnow('COM10')  # ESP1
q8 = q8_espnow('COM4')     # ESP2
q8.enable_torque()

pos_x = leg.d/2
pos_y = (leg.l1 + leg.l2) * 0.667
q1_idle, q2_idle, success = leg.ik_solve(pos_x, pos_y, True, 1)
q8.move_mirror([q1_idle, q2_idle], 1000)
time.sleep(2)

while True:
    clock.tick(SPEED)
    pygame.event.get()
    keys = pygame.key.get_pressed()

    if movement:
        if keys[pygame.K_w]:
            if not ongoing:
                move_list = movement_start(leg, 'f')
            pos, move_list = movement_tick(move_list)
            q8.move_all(pos, 0)
        elif keys[pygame.K_s]:
            if not ongoing:
                move_list = movement_start(leg, 'b')
            pos, move_list = movement_tick(move_list)
            q8.move_all(pos, 0)
        elif keys[pygame.K_a]:
            if not ongoing:
                move_list = movement_start(leg, 'l')
            pos, move_list = movement_tick(move_list)
            q8.move_all(pos, 0)
        elif keys[pygame.K_d]:
            if not ongoing:
                move_list = movement_start(leg, 'r')
            pos, move_list = movement_tick(move_list)
            q8.move_all(pos, 0)
        else:
            q8.move_mirror([q1_idle, q2_idle], 0)
            ongoing = False
            movement = False
    else:
        if (keys[pygame.K_w] or keys[pygame.K_a] or 
            keys[pygame.K_s] or keys[pygame.K_d]):
            movement = True
        elif keys[pygame.K_b]:
            voltage = q8.check_voltage()
            print("Battery Voltage: %.1f" % (voltage))
            time.sleep(0.2)
        elif keys[pygame.K_j]:
            print("Jump")
            
            jump(JUMP_3, 0.08)
            time.sleep(0.5)
            q8.move_mirror([q1_idle, q2_idle], 500)
        elif keys[pygame.K_ESCAPE]:
            break

q8.disable_torque()
pygame.quit()

# if __name__ == "__main__":
#     main()