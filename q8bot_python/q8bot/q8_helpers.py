'''
Written by yufeng.wu0902@gmail.com

Helper functions for q8_operate.py.
'''

import math
import matplotlib.pyplot as plt
from kinematics_solver import *
from q8_espnow import *

def generate_walk(leg, dir, x0 = 9.75, y0 = 40, xrange = 20.0, 
                        yrange = 20.0, lift_count = 10, len_factor = 8):
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
    # visualize_combined(y_list, move_p)
    # visualize_joints(move_p)

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

def parse_data(data):
    cur_1 = []
    pos_1 = []
    cur_2 = []
    pos_2 = []
    for i in range(0, len(data), 4):
        cur_1.append(data[i])
        pos_1.append(data[i+1])
        cur_2.append(data[i+2])
        pos_2.append(data[i+3])
    cur_1 = np.array(cur_1) - 10000
    cur_2 = np.array(cur_2) - 10000
    pos_1 = np.round(dxl2rad(np.array(pos_1)),2)
    pos_2 = np.round(dxl2rad(np.array(pos_2)),2)
    print(cur_1)
    print(pos_1)
    print(cur_2)
    print(pos_1)

    # Generate x axis timestamps for the data
    t = [i for i in range(len(cur_1))]
    # Create a 2x1 subplot
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    ax1.plot(t, cur_1, label='Joint 1', color='tab:blue')  # Blue color for the gait trajectory
    ax1.plot(t, cur_2, label='Joint 2', color='tab:orange')  # Blue color for the gait trajectory
    ax1.set_ylabel('Joint Current (mA)', fontsize=15, labelpad=15)  # Move y label further
    ax1.legend(loc='upper right',fontsize=14)
    # ax1.set_aspect(0.5)
    ax2.plot(t, pos_1, label='Joint 1', color='tab:green')  # Red color for q_1
    ax2.plot(t, pos_2, label='Joint 2', color='tab:red')  # Green color for q_2
    ax2.set_xlabel('Time Stamp', fontsize=15, labelpad=15)  # Move x label further
    ax2.set_ylabel('Joint positions (rad)', fontsize=15, labelpad=15)  # Move y label further
    ax2.legend(loc='upper right',fontsize=14)
    # ax2.set_aspect(0.5)
    # Show the plot
    plt.tight_layout()
    plt.show()

def dxl2rad(angle_dxl):
    # Dynamixel joint 0 to 360 deg is 0 to 4096
    friendly_per_dxl = 360.0 / 4096.0
    angle_friendly = (angle_dxl - 4096) * friendly_per_dxl
    return angle_friendly * math.pi / 180

def visualize_gait(y_list):
    y_list = y_list[-int(len(y_list)/2):] + (y_list * 5)
    x_list = [i for i in range(len(y_list))]
    # Plot the data
    plt.plot(x_list, y_list)
    # Label the axes
    plt.xlabel('Time Stamp')
    plt.ylabel('Y Coordinate of Trajectory (mm)')
    # Title for the plot
    plt.title('Robot Trajectory and Joint Angles')
    # Show the plot
    plt.show()

def visualize_combined(y_list, q_list):
    # Visualizing gait trajectory
    y_list = y_list[-int(len(y_list)/2):] + (y_list * 5)
    x_list = [i for i in range(len(y_list))]
    
    # Create a 2x1 subplot grid (2 rows, 1 column)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

    # First subplot: Plot gait trajectory
    ax1.plot(x_list, y_list)  # Blue color for the gait trajectory
    ax1.set_ylabel('Y Coordinate (mm)', fontsize=15, labelpad=15)  # Move y label further

    # Visualizing joint angles
    q_1 = [q_val[0] for q_val in q_list]
    q_2 = [q_val[1] for q_val in q_list]
    q_1 = q_1[-int(len(q_1)/2):] + (q_1 * 5)
    q_2 = q_2[-int(len(q_2)/2):] + (q_2 * 5)

    # Convert joint angles to radians
    q_1 = [math.radians(deg) for deg in q_1]
    q_2 = [math.radians(deg) for deg in q_2]
    
    # Second subplot: Plot joint angles with different colors
    ax2.plot(x_list, q_1, label='q_1 (rad)', color='tab:orange')  # Red color for q_1
    ax2.plot(x_list, q_2, label='q_2 (rad)', color='tab:green')  # Green color for q_2
    ax2.set_xlabel('Time Stamp', fontsize=15, labelpad=15)  # Move x label further
    ax2.set_ylabel('Joint angles (rad)', fontsize=15, labelpad=15)  # Move y label further
    
    # Display the legend
    ax2.legend(loc='upper right',fontsize=14)

    # Show the plot
    plt.tight_layout()
    plt.show()