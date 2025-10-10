import csv
import numpy as np
from helpers import *
import matplotlib.pyplot as plt


# Load the text file and convert to CSV
# input_file = "cpp_data_2.txt"
# output_file = "cpp_data_2.csv"
input_file = "py_data_2.txt"
output_file = "py_data_2.csv"

with open(input_file, 'r') as f:
    content = f.read()

# Split by commas and structure as rows (e.g., every 5 numbers per row)
# data = content.strip().split()
data = content.strip().split(',')
data = [n.strip() for n in data if n.strip()]  # remove whitespace and empty items
# arr = np.array(numbers, dtype=float)
# print(arr)
cur_1, cur_2, pos_1, pos_2 = [], [], [], []
complete_len = len(data) - (len(data) % 4)  # Only keep complete packets of 4
data = data[:complete_len]
for i in range(0, len(data), 4):
    cur_1.append(data[i])
    pos_1.append(data[i+1])
    cur_2.append(data[i+2])
    pos_2.append(data[i+3])
cur_1 = np.array(cur_1, dtype=float) - 10000
cur_2 = np.array(cur_2, dtype=float) - 10000
pos_1 = np.round(dxl2rad(np.array(pos_1, dtype=float)),2)
pos_2 = np.round(dxl2rad(np.array(pos_2, dtype=float)),2)
print(f"cur1: {cur_1}, pos1: {pos_1}, cur2: {cur_2}, pos2: {pos_2}")
# Generate x axis timestamps for the data
t = [i for i in range(len(cur_1))]
# Create a 3x1 subplot
fig, (ax2, ax3) = plt.subplots(2, 1, figsize=(16, 9))
ax2.plot(t, pos_1, label='Joint 1 measured', color='tab:red')
ax2.plot(t, pos_2, label='Joint 2 measured', color='tab:purple')
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


# Writing to CSV
# Define number of values per row in the CSV
row_length = 4  # Change this as needed

# Chunk the list into rows
rows = [data[i:i + row_length] for i in range(0, len(data), row_length)]

with open(output_file, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerows(rows)

print(f"CSV saved to {output_file}")