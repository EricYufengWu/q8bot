# Note: this program assumes the DXL motors are set to "Time-based Profile" in the Drive Mode register (ADDR 10)!

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

from dynamixel_sdk import *

PROTOCOL_VERSION            = 2.0

# Dynamixsl X Series relevant registers
ADDR_TORQUE_ENABLE          = 64
ADDR_PROFILE_ACCELERATION   = 108
ADDR_PROFILE_VELOCITY       = 112
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132
BAUDRATE                    = 57600

# Use the actual port assigned to the U2D2.
DEVICENAME                  = 'COM3'

TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold

# Custom instances
JOINTS                      = [11, 12]
GEAR_RATIO                  = 1.0   # 5 (output gear) to 4 (DXL input gear)

# Initialize PortHandler and PacketHandler instance
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def angle_dxl_to_friendly(angle_dxl):
    # Friendly units = deg
    # Dynamixel joint 0 to 360 deg is 0 to 4096
    friendly_per_dxl = 360.0 / 4096.0 / GEAR_RATIO
    angle_friendly = angle_dxl * friendly_per_dxl
    return angle_friendly

def angle_friendly_to_dxl(angle_friendly):
    # Friendly units = deg
    # Dynamixel joint 00 to 360 deg is 0 to 4096
    friendly_per_dxl = 360.0 / 4096.0 / GEAR_RATIO
    angle_dxl = int(angle_friendly / friendly_per_dxl + 0.5)
    return angle_dxl

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()
    
# Enable Dynamixel Torque
for joint in JOINTS:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, joint, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("joint ", joint, ": %s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("joint ", joint, ": %s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Joint {} has been successfully connected".format(joint))

# Main Loop
while(1):
    print("Press any key to continue (or press ESC to quit)")
    if getch() == '\x1b':    #esc key
        break

    time.sleep(4)

    for x in range(5):
        # Set Dynamixel time-based profiles
        dur_ms = 500
        for i in range(len(JOINTS)):
            packetHandler.write4ByteTxRx(portHandler, JOINTS[i], ADDR_PROFILE_VELOCITY, dur_ms)
            packetHandler.write4ByteTxRx(portHandler, JOINTS[i], ADDR_PROFILE_ACCELERATION, int(dur_ms / 3))
        # Move Dynamixel motors
        goal_pos = [angle_friendly_to_dxl(75), angle_friendly_to_dxl(75)]
        for i in range(len(JOINTS)):
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, JOINTS[i], ADDR_GOAL_POSITION, goal_pos[i])
        time.sleep(0.5)

        # Set Dynamixel time-based profiles
        dur_ms = 0
        for i in range(len(JOINTS)):
            packetHandler.write4ByteTxRx(portHandler, JOINTS[i], ADDR_PROFILE_VELOCITY, dur_ms)
            packetHandler.write4ByteTxRx(portHandler, JOINTS[i], ADDR_PROFILE_ACCELERATION, int(dur_ms / 3))
        # Move Dynamixel motors
        goal_pos = [angle_friendly_to_dxl(-10), angle_friendly_to_dxl(-10)]
        for i in range(len(JOINTS)):
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, JOINTS[i], ADDR_GOAL_POSITION, goal_pos[i])
        time.sleep(0.04)
        goal_pos = [angle_friendly_to_dxl(45), angle_friendly_to_dxl(45)]
        for i in range(len(JOINTS)):
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, JOINTS[i], ADDR_GOAL_POSITION, goal_pos[i])
        time.sleep(0.5)
