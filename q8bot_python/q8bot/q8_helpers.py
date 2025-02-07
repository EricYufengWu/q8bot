'''
Written by yufeng.wu0902@gmail.com

Helper functions for q8_operate.py.
'''

import math
# import matplotlib.pyplot as plt
from kinematics_solver import *
from q8_espnow import *

def generate_walk(leg, dir, x0 = 9.75, y0 = 40, xrange = 20.0, 
                        yrange = 20.0, lift_count = 10, len_factor = 12):
    move_p, move_n = [], []
    x_p, x_n = x0 - xrange / 2, x0 + xrange / 2
    x_lift_step = xrange / lift_count
    x_down_step = xrange / lift_count / (len_factor - 1)
    q1_d, q2_d, success = leg.ik_solve(x0, y0, True, 1)

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
                return generate_walk(leg, dir, x0, y0, xr_new, yr_new)
            return dummy_movement(q1_d, q2_d)
        move_p.append([q1_p, q2_p])
        move_n.append([q1_n, q2_n])

    split = int(lift_count * len_factor/4)
    move_p2 = move_p[split:] + move_p[:split]
    move_p3 = move_p[split*2:] + move_p[:split*2]
    move_p4 = move_p[split*3:] + move_p[:split*3]
    move_n2 = move_n[split:] + move_n[:split]
    move_n3 = move_n[split*2:] + move_n[:split*2]
    move_n4 = move_n[split*3:] + move_n[:split*3]

    if dir == 'f':
        return append_pos_list(move_p, move_p2, move_p3, move_p4)
    elif dir == 'r':
        return append_pos_list(move_p, move_n2, move_p3, move_n4)
    elif dir == 'l':
        return append_pos_list(move_n, move_p2, move_n3, move_p4)
    else: # all other conditions default to "back"
        return append_pos_list(move_n, move_n2, move_n3, move_n4)

def generate_amber(leg, dir, x0 = 9.75, y0 = 40, xrange = 20.0, 
                        yrange = 20.0, lift_count = 10, len_factor = 3):
    # Generate two sets of single-leg position lists (forward + backward). 
    # Mix and match to create an aggregated list for 4 legs.
    move_p, move_ps, move_n = [], [], []
    x_list_p, x_list_n, x_p_s, y_list = [], [], [], []
    x_p, x_n = x0 - xrange / 2, x0 + xrange / 2
    x_ps = x_p * 0.5 + 9.75 *0.5
    x_lift_step = xrange / lift_count
    x_down_step = xrange / lift_count / (len_factor - 1)
    q1_d, q2_d, success = leg.ik_solve(x0, y0, True, 1)

    if y0 - yrange < 5:
        print("Invalid: y below physical limit.")
        return dummy_movement(q1_d, q2_d)

    for i in range(lift_count * len_factor):
        if i < lift_count:
            x_p += x_lift_step * min(i, 1)
            x_ps += x_lift_step * min(i, 1) * 0.5
            x_n -= x_lift_step * min(i, 1)
            y = y0 - math.sin((i+1)*(math.pi/(lift_count))) * yrange
        else:
            x_p = x_p - x_down_step
            x_ps = x_ps - x_down_step * 0.5
            x_n = x_n + x_down_step
            y = y0
        q1_p, q2_p, check1 = leg.ik_solve(x_p, y, True, 1)
        q1_ps, q2_ps, check3 = leg.ik_solve(x_ps, y, True, 1) # smaller gait
        q1_n, q2_n, check2 = leg.ik_solve(x_n, y, True, 1)
        # Making a list of x y for visualization
        x_list_p.append(x_p)
        x_list_n.append(x_n)
        x_p_s.append(x_ps)
        y_list.append(y)
        #
        if len(str(q1_p))>5 or len(str(q2_p))>5:
            print("Invalid: ",x_p, x_n, y, check1, check2)
            xr_new, yr_new = xrange - 1, yrange - 1
            print(xr_new, yr_new)
            if xr_new > 0 and yr_new > 0:
                print("Retry with smaller step size")
                return generate_amber(leg, dir, x0, y0, xr_new, yr_new)
            return dummy_movement(q1_d, q2_d)
        move_p.append([q1_p, q2_p])
        move_ps.append([q1_ps, q2_ps])
        move_n.append([q1_n, q2_n])
    # visualize_gait([x_list_p],[y_list])

    split = int(lift_count * len_factor/2)
    move_p2 = move_p[split:] + move_p[:split]
    move_ps2 = move_ps[split:] + move_ps[:split]
    move_n2 = move_n[split:] + move_n[:split]
    
    if dir == 'f':
        return append_pos_list(move_p, move_p2, move_p2, move_p)
    elif dir == 'r':
        return append_pos_list(move_p, move_n2, move_p2, move_n)
    elif dir == 'l':
        return append_pos_list(move_n, move_p2, move_n2, move_p)
    elif dir == 'fr':
        return append_pos_list(move_p, move_ps2, move_p2, move_ps)
    elif dir == 'fl':
        return append_pos_list(move_ps, move_p2, move_ps2, move_p)
    else: # all other conditions default to "back"
        return append_pos_list(move_n, move_n2, move_n2, move_n)
    
def generate_gallop(leg, dir, x0 = 9.75, y0 = 40, xrange = 20.0, 
                        yrange = 20.0, lift_count = 5, len_factor = 4):
    move_p, move_n = [], []
    x_list_p, x_list_n, y_list = [], [], []
    x_p, x_n = x0 - xrange / 2, x0 + xrange / 2
    x_lift_step = xrange / lift_count
    x_down_step = xrange / lift_count / (len_factor - 1)
    q1_d, q2_d, success = leg.ik_solve(x0, y0, True, 1)

    if y0 - yrange < 5:
        print("Invalid: y below physical limit.")
        return dummy_movement(q1_d, q2_d)

    for i in range(lift_count * len_factor):
        if i < lift_count:
            x_p += x_lift_step * min(i, 1)
            x_n -= x_lift_step * min(i, 1)
            y = y0 + math.sin((i+1)*(math.pi/(lift_count))) * yrange
        else:
            x_p = x_p - x_down_step
            x_n = x_n + x_down_step
            y = y0
        q1_p, q2_p, check1 = leg.ik_solve(x_p, y, True, 1)
        q1_n, q2_n, check2 = leg.ik_solve(x_n, y, True, 1)
        # Making a list of x y for visualization
        x_list_p.append(x_p)
        x_list_n.append(x_n)
        y_list.append(y)
        if len(str(q1_p))>5 or len(str(q2_p))>5:
            print("Invalid: ",x_p, x_n, y, check1, check2)
            xr_new, yr_new = xrange - 1, yrange - 1
            print(xr_new, yr_new)
            if xr_new > 0 and yr_new > 0:
                print("Retry with smaller step size")
                return generate_gallop(leg, dir, x0, y0, xr_new, yr_new)
            return dummy_movement(q1_d, q2_d)
        move_p.append([q1_p, q2_p])
        move_n.append([q1_n, q2_n])
    # visualize_gait([x_list_p],[y_list])
    
    split = int(lift_count * len_factor/4)
    move_p2 = move_p[split:] + move_p[:split]
    move_n2 = move_n[split:] + move_n[:split]
    
    if dir == 'f':
        return append_pos_list(move_p, move_p, move_p2, move_p2)
    else: # all other conditions default to "back"
        return append_pos_list(move_n, move_n, move_n2, move_n2)

def generate_pronk(leg, dir, x0 = 9.75, y0 = 40, xrange = 20.0, 
                        yrange = 20.0, lift_count = 5, len_factor = 10):
    move_p, move_n = [], []
    x_list_p, x_list_n, y_list = [], [], []
    x_p, x_n = x0 - xrange / 2, x0 + xrange / 2
    x_lift_step = xrange / lift_count
    x_down_step = xrange / lift_count / (len_factor - 1)
    q1_d, q2_d, success = leg.ik_solve(x0, y0, True, 1)

    if y0 - yrange < 5:
        print("Invalid: y below physical limit.")
        return dummy_movement(q1_d, q2_d)

    for i in range(lift_count * len_factor):
        if i < lift_count:
            x_p += x_lift_step * min(i, 1)
            x_n -= x_lift_step * min(i, 1)
            y = y0 + math.sin((i+1)*(math.pi/(lift_count))) * yrange
        else:
            x_p = x_p - x_down_step
            x_n = x_n + x_down_step
            y = y0
        q1_p, q2_p, check1 = leg.ik_solve(x_p, y, True, 1)
        q1_n, q2_n, check2 = leg.ik_solve(x_n, y, True, 1)
        # Making a list of x y for visualization
        x_list_p.append(x_p)
        x_list_n.append(x_n)
        y_list.append(y)
        if len(str(q1_p))>5 or len(str(q2_p))>5:
            print("Invalid: ",x_p, x_n, y, check1, check2)
            xr_new, yr_new = xrange - 1, yrange - 1
            print(xr_new, yr_new)
            if xr_new > 0 and yr_new > 0:
                print("Retry with smaller step size")
                return generate_pronk(leg, dir, x0, y0, xr_new, yr_new)
            return dummy_movement(q1_d, q2_d)
        move_p.append([q1_p, q2_p])
        move_n.append([q1_n, q2_n])
    # visualize_gait([x_list_p],[y_list])

    if dir == 'f':
        return append_pos_list(move_p, move_p, move_p, move_p)
    else: # all other conditions default to "back"
        return append_pos_list(move_n, move_n, move_n, move_n)

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

def dummy_movement(q1 = 90, q2 = 90):
    # Movement placeholder for the types I have yet to write atual code for :/
    return[[q1 if i % 2 == 0 else q2 for i in range(8)] for j in range(10)]

def movement_tick(move_list):
    # For each step in a movement, cycle through pre-calculated list.
    pos = move_list[0]
    new_list = move_list[1:] + [move_list[0]]
    return pos, new_list

# def visualize_gait(x_list, y_list):
#     for i in range(len(x_list)):
#         plt.plot(x_list[i], y_list[i],'o', label=i)
#     plt.legend()
#     plt.show()