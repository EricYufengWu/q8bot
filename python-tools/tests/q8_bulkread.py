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
ADDR_BYTE_LEN               = 4
BAUDRATE                    = 1000000

# Use the actual port assigned to the U2D2.
DEVICENAME                  = 'COM3'

TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold

# Custom instances
JOINTS                      = [11, 12, 13, 14, 15, 16, 17, 18]
GEAR_RATIO                  = 1.0
ZERO_OFFSET                 = 4096  # Increase avoid dealing with large numbers
                                    # at negative angle. Change DXL config.
pos_raw                     = [0, 0, 0, 0, 0, 0, 0, 0]
pos_deg                     = [0, 0, 0, 0, 0, 0, 0, 0]

# Helper Functions
def angle_dxl_to_friendly(angle_dxl):
    # Friendly units = deg
    # Dynamixel joint 0 to 360 deg is 0 to 4096
    friendly_per_dxl = 360.0 / 4096.0 / GEAR_RATIO
    angle_friendly = (angle_dxl - ZERO_OFFSET) * friendly_per_dxl
    return angle_friendly

def angle_friendly_to_dxl(angle_friendly):
    # Friendly units = deg
    # Dynamixel joint 00 to 360 deg is 0 to 4096
    friendly_per_dxl = 360.0 / 4096.0 / GEAR_RATIO
    angle_dxl = int(angle_friendly / friendly_per_dxl + 0.5) + ZERO_OFFSET
    return angle_dxl

# Main code
def main():
    # Initialize kinematics solver
    leg = k_solver()

    # Initialize PortHandler and PacketHandler instance
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    groupBulkRead = GroupBulkRead(portHandler, packetHandler)

    # Open port and set baudrate
    if portHandler.openPort() and portHandler.setBaudRate(BAUDRATE):
        print("Connection established")
    else:
        print("Failed to open the port")
        quit()
        
    # Disable Dynamixel Torque
    for joint in JOINTS:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
            portHandler, 
            joint, 
            ADDR_TORQUE_ENABLE, 
            TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            print("Torque off failed")
    print("Torque off success")

    # Add parameter storage for bulk read
    for joint in JOINTS:
        dxl_addparam_result = groupBulkRead.addParam(
            joint, 
            ADDR_PRESENT_POSITION, 
            ADDR_BYTE_LEN)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupBulkRead addparam failed" % joint)

    while(1):
        # Bulkread motor present positions
        dxl_comm_result = groupBulkRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Check if groupbulkread data of Dynamixel is available
        for joint in JOINTS:
            dxl_getdata_result = groupBulkRead.isAvailable(
                joint, 
                ADDR_PRESENT_POSITION, 
                ADDR_BYTE_LEN)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupBulkRead getdata failed" % joint)
                quit()
        
        # Check if groupbulkread data of Dynamixel#2 is available
        for joint in JOINTS:
            dxl_getdata_result = groupBulkRead.isAvailable(
                joint, 
                ADDR_PRESENT_POSITION, 
                ADDR_BYTE_LEN)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupBulkRead getdata failed" % joint)
                quit()

        # Get present position value
        for j in range(len(JOINTS)):
            pos_deg[j] = angle_dxl_to_friendly(groupBulkRead.getData(
                JOINTS[j], 
                ADDR_PRESENT_POSITION, 
                ADDR_BYTE_LEN))
            
        pos_x, pos_y = leg.fk_solve(pos_deg[0], pos_deg[1])
        print("[ID:011] %.1f  [ID:012] %.1f  [ID:013] %.1f  [ID:014] %.1f  [ID:015] %.1f  [ID:016] %.1f  [ID:017] %.1f  [ID:018] %.1f" % 
              (pos_deg[0], pos_deg[1], pos_deg[2], pos_deg[3], pos_deg[4], pos_deg[5], pos_deg[6], pos_deg[7]))
        
        time.sleep(0.01)

if __name__ == "__main__":
    main()
