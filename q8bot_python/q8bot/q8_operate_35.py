# This is the latest control script for Q8bot (using ESPNow)

import time
import math
import pygame
from kinematics_solver import *
from q8_espnow import *

# User-modifiable constants
SPEED = 100
res = 0.2
y_min = 15
# PORT = 'COM6'    #ESP1
PORT = 'COM4'     #ESP2

# Jumping parameters. Change this to tune jumping brhavior
JUMP_LOW = [12, 168, 12, 168, 12, 168, 12, 168]         # 35mm leg
JUMP_1 = [95, 85, 95, 85, 95, 85, 95, 85]
JUMP_2 = [95, 85, 95, 85, -25, 205, -25, 205]
JUMP_REST = [20, 160, 20, 160, 20, 160, 20, 160]        # 35mm leg

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

def generate_diag_amber(leg, dir, x0 = 9.75, y0 = 40, xrange = 20.0, 
                        yrange = 20.0, lift_count = 10, len_factor = 3):
    # Generate two sets of single-leg position lists (forward + backward). 
    # Mix and match to create an aggregated list for 4 legs.
    move_p, move_n = [], []
    x_p, x_n = x0 - xrange / 2, x0 + xrange / 2
    x_lift_step = xrange / lift_count
    x_down_step = xrange / lift_count / (len_factor - 1)
    q1_d, q2_d, success = leg.ik_solve(x0, y0, True, 1)

    # (y - yrange) cannot be lower than the robot's physical limit.
    if y0 - yrange < 5:
        print("Invalid: y below physical limit.")
        return dummy_movement(q1_d, q2_d)

    for i in range(lift_count * len_factor):
        if i < lift_count:
            x_p += x_lift_step * min(i, 1)
            x_n -= x_lift_step * min(i, 1)
            y = y0 - math.sin((i+1)*(math.pi/(lift_count+1))) * yrange
        else:
            x_p -= x_down_step
            x_n += x_down_step
            y = y0
        q1_p, q2_p, check1 = leg.ik_solve(x_p, y, True, 1)
        q1_n, q2_n, check2 = leg.ik_solve(x_n, y, True, 1)
        if len(str(q1_p))>5 or len(str(q2_p))>5:
            print("Invalid: ",x_p, x_n, y, check1, check2)
            xr_new, yr_new = xrange - 1, yrange - 1
            print(xr_new, yr_new)
            if xr_new > 0 and yr_new > 0:
                print("Retry with smaller step size")
                return generate_diag_amber(leg, dir, x0, y0, xr_new, yr_new)
            return dummy_movement(q1_d, q2_d)
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

def movement_start(leg, dir, x_0, y_0, x_size, y_size, move_type = 'AMBER'):
    # start movement by generating the position list
    global ongoing
    ongoing = True
    if move_type == 'WALK':
        return dummy_movement()
    elif move_type == 'AMBER':
        return generate_diag_amber(leg, dir, x_0, y_0, x_size, y_size)
    elif move_type == 'TROT':
        return dummy_movement()
    elif move_type == 'GALLOP':
        return dummy_movement()
    else:
        return dummy_movement()

def dummy_movement(q1 = 90, q2 = 90):
    # Movement placeholder for the types I have yet to write atual code for :/
    return[[q1 if i % 2 == 0 else q2 for i in range(8)] for j in range(10)]

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

leg = k_solver(19.5, 25, 35, 25, 35) # 35mm leg
q8 = q8_espnow(PORT)
q8.enable_torque()

# Starting location of leg end effector in x and y
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
                move_list = movement_start(leg, 'f', pos_x, pos_y, step_size, step_size)
            pos, move_list = movement_tick(move_list)
            q8.move_all(pos, 0)
        elif keys[pygame.K_s]:
            if not ongoing:
                move_list = movement_start(leg, 'b', pos_x, pos_y, step_size, step_size)
            pos, move_list = movement_tick(move_list)
            q8.move_all(pos, 0)
        elif keys[pygame.K_a]:
            if not ongoing:
                move_list = movement_start(leg, 'l', pos_x, pos_y, step_size, step_size)
            pos, move_list = movement_tick(move_list)
            q8.move_all(pos, 0)
        elif keys[pygame.K_d]:
            if not ongoing:
                move_list = movement_start(leg, 'r', pos_x, pos_y, step_size, step_size)
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
            move_xy(pos_x, pos_y, 500)
            move_xy(pos_x, pos_y, 500)
            time.sleep(1)
        elif keys[pygame.K_ESCAPE]:
            break

q8.disable_torque()
pygame.quit()

# if __name__ == "__main__":
#     main()