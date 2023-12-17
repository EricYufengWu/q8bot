# Note: this program assumes the DXL motors are set to "Time-based Profile" in the Drive Mode register (ADDR 10)!

import time
from dynamixel_sdk import *
from kinematics_solver import *

PROTOCOL_VERSION            = 2.0

# Dynamixsl X Series relevant registers
ADDR_TORQUE_ENABLE          = 64
ADDR_PROFILE_ACCELERATION   = 108
ADDR_PROFILE_VELOCITY       = 112
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132
BAUDRATE                    = 1000000

# Use the actual port assigned to the U2D2.
DEVICENAME                  = 'COM3'

TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold

# Custom instances
JOINTS                      = [11, 12]
GEAR_RATIO                  = 1.0
ZERO_OFFSET                 = 4096  # Increase avoid dealing with large numbers at negative angle. Change DXL config.
pos_raw                     = [0, 0]
pos_deg                     = [0, 0]

# Helper Functions
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

# Main code
def main():
    # Initialize kinematics solver
    leg = k_solver()

    # Initialize PortHandler and PacketHandler instance
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # Open port and set baudrate
    if portHandler.openPort() and portHandler.setBaudRate(BAUDRATE):
        print("Connection established")
    else:
        print("Failed to open the port")
        quit()
        
    # Disable Dynamixel Torque
    for joint in JOINTS:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, joint, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("joint ", joint, ": %s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("joint ", joint, ": %s" % packetHandler.getRxPacketError(dxl_error))
    print("Torque off")

    while(1):
        for i in range(len(JOINTS)):
            try:
                pos_raw[i], dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, JOINTS[i], ADDR_PRESENT_POSITION)
                pos_raw[i] -= ZERO_OFFSET
                pos_deg[i] = angle_dxl_to_friendly(pos_raw[i])
            except:
                print("joint ", joint, ": %s" % packetHandler.getTxRxResult(dxl_comm_result))
                print("joint ", joint, ": %s" % packetHandler.getRxPacketError(dxl_error))
        pos_x, pos_y = leg.fk_solve(pos_deg[0], pos_deg[1])
        print("[ID:011] PresPos:%.1f  [ID:012] PresPos:%.1f  [X,Y] %.1f, %.1f" % (pos_deg[0], pos_deg[1], pos_x, pos_y))
        time.sleep(0.01)

if __name__ == "__main__":
    main()
