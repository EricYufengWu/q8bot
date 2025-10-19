'''
Written by yufeng.wu0902@gmail.com

Helper functions for q8_operate.py.
'''

import math
import matplotlib.pyplot as plt
from kinematics_solver import *
from espnow import *

# Q8bot range of motion via points
R1 = [100, 80, 100, 80, 100, 80, 100, 80]
R2 = [0, 45, 0, 45, 0, 45, 0, 45]
R3 = [-90, 45, -90, 45, -90, 45, -90, 45]
R4 = [-20, 200, -20, 200, -20, 200, -20, 200]
R5 = [130, 270, 130, 270, 130, 270, 130, 270]
R6 = [130, 180, 130, 180, 130, 180, 130, 180]
R7 = [100, 80, 100, 80, 100, 80, 100, 80]
R8 = [-20, 200, -20, 200, -20, 200, -20, 200]
R9 = [30, 150, 30, 150, 30, 150, 30, 150]
R = [R1, R2, R3, R4, R5, R6, R7, R8, R9]

# Q8bot greet motion via points
G1 = [-90, 45, -90, 45, -90, 45, -90, 45]
G2 = [0, 45, 0, 45, 0, 45, 0, 45]
G3 = [-45, 45, -45, 45, 50, 75, 50, 75]
G4 = [45, 90, -45, 45, 50, 75, 50, 75]
G5 = [-45, 45, 45, 90, 50, 75, 50, 75]
G6 = [-90, 45, -90, 45, -90, 45, -90, 45]

def dummy_movement(q1 = 90, q2 = 90):
    # Movement placeholder for the types I have yet to write atual code for :/
    return[[q1 if i % 2 == 0 else q2 for i in range(8)] for j in range(10)], []

def movement_tick(move_list):
    # For each step in a movement, cycle through pre-calculated list.
    pos = move_list[0]
    new_list = move_list[1:] + [move_list[0]]
    # for i in range(len(pos)):
    #     pos[i] = round(pos[i])
    # print(pos)
    return pos, new_list

def parse_data(data, cmd, y_list):
    cur_1, cur_2, pos_1, pos_2 = [], [], [], []
    for i in range(0, len(data), 4):
        cur_1.append(data[i])
        pos_1.append(data[i+1])
        cur_2.append(data[i+2])
        pos_2.append(data[i+3])
    cur_1 = np.array(cur_1) - 10000
    cur_2 = np.array(cur_2) - 10000
    pos_1 = np.round(dxl2rad(np.array(pos_1)),2)
    pos_2 = np.round(dxl2rad(np.array(pos_2)),2)
    cmd_1 = [q_val[0] for q_val in cmd]
    cmd_2 = [q_val[1] for q_val in cmd]
    y_length, d_length = len(y_list), len(cur_1)
    if d_length <= y_length:
        y_new = y_list[:d_length]  # Take only the first n elements of y_list
        cmd_1, cmd_2 = cmd_1[:d_length], cmd_2[:d_length]
    else:
        y_new = np.tile(y_list, (d_length // y_length) + 1)[:d_length]  # Repeating and slicing to fit length n
        cmd_1 = np.tile(cmd_1, (d_length // y_length) + 1)[:d_length]
        cmd_2 = np.tile(cmd_2, (d_length // y_length) + 1)[:d_length]
    cmd_1 = [math.radians(deg) for deg in cmd_1]
    cmd_2 = [math.radians(deg) for deg in cmd_2]

    # Generate x axis timestamps for the data
    t = [i for i in range(len(cur_1))]
    # Create a 3x1 subplot
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(16, 9))
    ax1.plot(t, y_new, label='y', color='tab:blue') 
    ax1.set_ylabel('Leg Trajectory Y (mm)', fontsize=15, labelpad=15)  # Move y label further
    ax2.plot(t, pos_1, label='Joint 1 measured', color='tab:red')
    ax2.plot(t, pos_2, label='Joint 2 measured', color='tab:purple')
    ax2.plot(t, cmd_1, label='Joint 1 desired', color='tab:brown', linestyle='--')
    ax2.plot(t, cmd_2, label='Joint 2 desired', color='tab:pink', linestyle='--')
    ax2.set_ylabel('Joint Positions (rad)', fontsize=15, labelpad=15)
    ax2.legend(loc='upper right',fontsize=13)
    ax3.plot(t, cur_1, label='Joint 1', color='tab:orange')
    ax3.plot(t, cur_2, label='Joint 2', color='tab:green')
    ax3.set_xlabel('Time Stamp', fontsize=15, labelpad=10)
    ax3.set_ylabel('Joint Current (mA)', fontsize=15)
    ax3.legend(loc='upper right',fontsize=13)
    # Show the plot
    plt.tight_layout()
    plt.show()

def dxl2rad(angle_dxl):
    # Dynamixel joint 0 to 360 deg is 0 to 4096
    friendly_per_dxl = 360.0 / 4096.0
    angle_friendly = (angle_dxl - 4096) * friendly_per_dxl
    return angle_friendly * math.pi / 180

def visualize_gaits(y1, y2, p1, p2):
    y1 = y1[-int(len(y1)/2):] + (y1 * 5)
    y2 = y2[-int(len(y2)/2):] + (y2 * 5)
    q_1 = [q_val[0] for q_val in p1]
    q_2 = [q_val[1] for q_val in p1]
    q_3 = [q_val[0] for q_val in p2]
    q_4 = [q_val[1] for q_val in p2]
    q1 = q_1[-int(len(q_1)/2):] + (q_1 * 5)
    q2 = q_2[-int(len(q_2)/2):] + (q_2 * 5)
    q3 = q_3[-int(len(q_3)/2):] + (q_3 * 5)
    q4 = q_4[-int(len(q_4)/2):] + (q_4 * 5)
    q1 = [math.radians(deg) for deg in q1]
    q2 = [math.radians(deg) for deg in q2]
    q3 = [math.radians(deg) for deg in q3]
    q4 = [math.radians(deg) for deg in q4]
    x1 = [i for i in range(len(y1))]
    x2 = [i for i in range(len(q1))]

    plt.rc('font', family='serif')
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))
    ax1.plot(x1, y1, label = 'Leg 1 & 4')
    ax1.plot(x1, y2, label = 'Leg 2 & 3')
    ax1.grid(True)
    ax1.set_aspect(3)
    ax1.set_ylabel('Y Trajectory (mm)', fontsize=14)
    ax1.legend(loc = 'lower right')
    ax2.plot(x2, q1, color='tab:blue', linewidth = 1.5, linestyle='dashdot')
    ax2.plot(x2, q2, color='tab:blue', linewidth = 1.5, linestyle='dashed')
    ax2.plot(x2, q3, color='tab:orange', linewidth = 1.5, linestyle='dashdot')
    ax2.plot(x2, q4, color='tab:orange', linewidth = 1.5, linestyle='dashed')
    ax2.grid(True)
    ax2.set_xlabel('Time Stamp', fontsize=14)
    ax2.set_ylabel('Joint angles (rad)', labelpad=3, fontsize=14)
    plt.tight_layout()
    # plt.legend()
    plt.show()

def visualize_traj(x_1, y_1, x_2, y_2):
    fig, axs = plt.subplots(1, 2, figsize=(12, 6))
    axs[0].plot(x_1, y_1, linewidth = 2, label='Sinusoid 1')
    axs[0].plot(x_2, y_2, linewidth = 2, label='Sinusoid 2')
    axs[0].legend(fontsize = 14)
    for i in range(len(x_1)):
        axs[1].plot(x_1[i], y_1[i],'o', markersize = 8, color = 'tab:blue')
    for i in range(len(x_2)):
        axs[1].plot(x_2[i], y_2[i],'.', markersize = 10, color = 'tab:orange')
    for ax in axs:
        ax.axis('equal')
        ax.set_xticks(np.arange(-10, 31, 10))
        ax.set_yticks(np.arange(0, 51, 10))
        ax.tick_params(axis='both', which='major', labelsize=14, width=2)  # Make ticks thicker
        ax.spines['top'].set_linewidth(2)   # Top axis line thickness
        ax.spines['right'].set_linewidth(2)  # Right axis line thickness
        ax.spines['left'].set_linewidth(2)   # Left axis line thickness
        ax.spines['bottom'].set_linewidth(2) # Bottom axis line thickness
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