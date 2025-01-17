# Note: this program assumes the DXL motors are set to "Time-based Profile" in the Drive Mode register (ADDR 10)!

import time
from dynamixel_sdk import *

PROTOCOL_VERSION            = 2.0

# Dynamixsl X Series relevant registers
ADDR_TORQUE_ENABLE          = 64
ADDR_PRESENT_CURRENT        = 126
LEN_PRESENT_CURRENT         = 2
ADDR_PRESENT_POSITION       = 132
LEN_PRESENT_POSITION        = 4
LEN_SYNC                    = 10
BAUDRATE                    = 1000000

# Use the actual port assigned to the U2D2.
DEVICENAME                  = 'COM5'

TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold

# Custom instances
JOINTS                      = [11, 12]
GEAR_RATIO                  = 1.0
ZERO_OFFSET                 = 4096  # Increase avoid dealing with large numbers
                                    # at negative angle. Change DXL config.


# Initialize PortHandler and PacketHandler instance
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
# Initialize GroupSyncRead instace for Present Position
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_CURRENT, LEN_SYNC)

def torque(value):
    for joint in JOINTS:
        packetHandler.write1ByteTxRx(portHandler, joint, ADDR_TORQUE_ENABLE, value)

# Open port and set baudrate
if portHandler.openPort() and portHandler.setBaudRate(BAUDRATE):
    print("Connection established")
else:
    print("Failed to open the port")
    quit()
    
# Disable Dynamixel Torque
torque(TORQUE_ENABLE)

# Add parameter storage for Dynamixel#1 present position value
for joint in JOINTS:
    dxl_addparam_result = groupSyncRead.addParam(joint)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncRead addparam failed" % joint)
        quit()

while(1):
    # Syncread present position
    dxl_comm_result = groupSyncRead.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Check if groupsyncread data is available
    for joint in JOINTS:
        dxl_getdata_result = groupSyncRead.isAvailable(joint, ADDR_PRESENT_CURRENT, LEN_SYNC)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % joint)
            quit()

    # Get Dynamixel#1 present position value
    pos_raw = []
    cur_list = []
    for joint in JOINTS:
        cur_list.append(groupSyncRead.getData(joint, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT))
        pos_raw.append(groupSyncRead.getData(joint, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION))

    print(cur_list, pos_raw)

    # Get compound value
    # value_read = groupSyncRead.getData(JOINTS[0], ADDR_PRESENT_CURRENT, 2)
    # value = value_read - 0x10000 if value_read > 0x8000 else value_read
    # print(value)

    time.sleep(0.01)