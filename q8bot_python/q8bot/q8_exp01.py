# Experiment 01: leg impedance control demonstration
# Robot leg will more to a neutral positon, with a built-in compliance using a
# PD controller. The compliance can then be tuned by changing the gains. When a
# key is pressed, the program will start recording current position of each
# for a pre-defined period of time, and then graph/save the result.

import os, time

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from q8_dynamixel import *
from kinematics_solver import *
import numpy as np
import matplotlib.pyplot as plt
import time
from datetime import datetime

def change_gain(leg, p_gain, d_gain):
    leg.joint_write4(leg.JOINTS[0], leg.ADDR_P_GAIN, p_gain)
    leg.joint_write4(leg.JOINTS[1], leg.ADDR_P_GAIN, p_gain)
    leg.joint_write4(leg.JOINTS[0], leg.ADDR_D_GAIN, d_gain)
    leg.joint_write4(leg.JOINTS[1], leg.ADDR_D_GAIN, d_gain)
    print(f"Gains changed. p: {p_gain}, d: {d_gain}")

def dxl_to_deg(angle_dxl):
    # Dynamixel joint 0 to 360 deg is 0 to 4096
    friendly_per_dxl = 360.0 / 4096.0
    angle_friendly = (angle_dxl - 4096) * friendly_per_dxl
    return angle_friendly

def format(array, bit_width=16):
    max_value = 2 ** bit_width
    signed_mask = 2 ** (bit_width - 1)
    format_results = np.where(array >= signed_mask, array - max_value, array)
    return abs(format_results)

def record_data(leg, duration = 1, freq = 100):
    interval = 1 / freq  # Time interval between reads
    num_samples = int(duration * freq)  # Total number of samples to record
    data = []
    timestamp = []

    start_time = time.time()
    for _ in range(num_samples):
        # Record individual data using bulkread
        # Need to use 2's complement to convert later
        # value = value_read - 0x10000 if value_read > 0x8000 else value_read
        data_raw, success = leg.syncread()
        timestamp.append(time.time() - start_time)
        # print(f"position: {data_raw[0]}, current: {data_raw[1]}")
        data.append(data_raw)
    end_time = time.time()
    print(f"Sampling rate: {round(num_samples / (end_time - start_time))}")
    return np.asarray(data), np.asarray(timestamp)

def calc_ik(deg1, deg2, ik):
    x = np.zeros_like(deg1)
    y = np.zeros_like(deg2)
    for i in range(deg1.shape[0]):
        x[i], y[i] =  ik.fk_solve(deg1[i], deg2[i])
    return x, y

def graph_data(data, time, ik):
    # Extract each element of the 2x2 matrix over time
    # time = np.arange(data.shape[0])
    pos1 = data[:, 0, 0]
    pos2 = data[:, 0, 1]
    vector_convert = np.vectorize(dxl_to_deg)
    pos1_deg = vector_convert(pos1)
    pos2_deg = vector_convert(pos2)
    x_pos, y_pos = calc_ik(pos1_deg, pos2_deg, ik)
    current1 = data[:, 1, 0]
    current2 = data[:, 1, 1]
    current1 = format(current1)
    current2 = format(current2)
    # Plot the lines
    fig, ax1 = plt.subplots(figsize=(10, 5))
    ax1.plot(time, y_pos, label='Leg Position', linestyle='-', marker=',')
    ax1.set_xlabel("Time (s)", labelpad=10, fontsize=12)
    ax1.set_ylabel("Joint Position (mm)", labelpad=10, fontsize=12)
    ax2 = ax1.twinx()
    ax2.plot(time, current1, label='Joint 1 Current', linestyle='-', marker=',', color='tab:orange')
    ax2.plot(time, current2, label='Joint 2 Current', linestyle='-', marker=',', color='tab:green')
    ax2.set_ylabel("Joint Current (mA)", labelpad=10, fontsize=12)
    # Add legends and title
    # plt.title("Joint Position and Current Over Time", fontsize=16)
    plt.tight_layout()

    # Combine handles and labels from both axes
    handles, labels = ax1.get_legend_handles_labels()
    handles2, labels2 = ax2.get_legend_handles_labels()
    combined_handles = handles + handles2
    combined_labels = labels + labels2
    # Add a single legend
    ax1.legend(combined_handles, combined_labels, loc="center right", fontsize=12)
    
    # Show the plot
    plt.show()
    return time, y_pos, current1, current2

def save_last_data(time, y_pos, current1, current2):
    # Generate the filename based on the current date and time
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    file_name = f"data_{timestamp}.csv"

    # Combine the arrays into a 2D array
    combined_data = np.column_stack((time, y_pos, current1, current2))

    # Save to CSV
    header = ['time', 'y_pos', 'current1', 'current2']
    np.savetxt(file_name, combined_data, delimiter=',', header=','.join(header), comments='', fmt='%g')

    print(f"Data successfully saved to {file_name}")

# Main code
def main():
    leg = q8_dynamixel('COM5', joint_list = [11, 12], baud = 1000000)
    ik = k_solver()
    rest_pos = [30, 150]
    p_gain = 400
    d_gain = 400

    leg.enable_torque()
    change_gain(leg, p_gain, d_gain)
    leg.move_all(rest_pos, 500)

    # Main Loop
    while(1):
        print("Press any key to continue (or press ESC to quit)")
        cmd = getch()
        if cmd == '=' and p_gain < 1000:
            p_gain += 50
            change_gain(leg, p_gain, d_gain)
        elif cmd == '-' and p_gain > 50:
            p_gain -= 50
            change_gain(leg, p_gain, d_gain)
        elif cmd == 's':
            print("start recording")
            data1, timestamp1 = record_data(leg, freq = 100)
            change_gain(leg, p_gain + 100, d_gain)
            data2, timestamp2 = record_data(leg, freq = 100)
            change_gain(leg, p_gain + 200, d_gain)
            data3, timestamp3 = record_data(leg, freq = 200)
            time, y_pos, current1, current2 = graph_data(np.concatenate((data1, data2, data3)), 
                       np.concatenate((timestamp1,timestamp2+2,timestamp3+4)), ik)

        elif cmd == '\x1b':    #esc key
            save_last_data(time, y_pos, current1, current2)
            break
    
    leg.disable_torque()

if __name__ == "__main__":
    main()