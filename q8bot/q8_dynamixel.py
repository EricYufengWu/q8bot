from dynamixel_sdk import *

class q8_dynamixel:
    def __init__(self, port, joint_list = [11, 12], baud = 1000000):
        self.DEVICENAME = port
        self.BAUDRATE = baud
        self.JOINTS = joint_list
        self.prev_profile = 0

        # Dynamixel constants
        self.PROTOCOL_VERSION = 2.0
        self.BYTE_LEN = 4
        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0
        self.DXL_MOVING_STATUS_THRESHOLD = 20
        self.ZERO_OFFSET = 4096
        self.GEAR_RATIO = 1

        # Dynamixsl X Series registers
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_PROFILE_ACCELERATION = 108
        self.ADDR_PROFILE_VELOCITY = 112
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_PRESENT_POSITION = 132
        
        # Initialize PortHandler and PacketHandler instance
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)
        self.groupBulkRead = GroupBulkRead(self.portHandler, 
                                           self.packetHandler)
        self.groupBulkWrite = GroupBulkWrite(self.portHandler, 
                                             self.packetHandler)
        self._start_comm()

    def enable_torque(self):
        for joint in self.JOINTS:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                self.portHandler, 
                joint, 
                self.ADDR_TORQUE_ENABLE, 
                self.TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                return False
        return True
    
    def disable_torque(self):
        for joint in self.JOINTS:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                self.portHandler, 
                joint, 
                self.ADDR_TORQUE_ENABLE, 
                self.TORQUE_DISABLE)
            if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                return False
        return True

    def move_leg(self, leg_pos, dur = 0):
        # Expects a pair of positions in deg. For example: [0, 90]
        if dur != self.prev_profile:
            self._set_profile(dur)
        for j in range(len(self.JOINTS)):
            self.groupBulkWrite.addParam(
                self.JOINTS[j], 
                self.ADDR_GOAL_POSITION, 
                self.BYTE_LEN, 
                self.param_goal_position(self._deg_to_dxl(leg_pos[j])))
        self.groupBulkWrite.txPacket()
        self.groupBulkWrite.clearParam()
        return
    
    #-------------------#
    # Private Functions #
    #-------------------#
    def _start_comm(self):
        # Open port and set baudrate
        if self.portHandler.openPort() and \
        self.portHandler.setBaudRate(self.BAUDRATE):
            return True
        else:
            return False

    def _dxl_to_deg(self, angle_dxl):
        # Dynamixel joint 0 to 360 deg is 0 to 4096
        friendly_per_dxl = 360.0 / 4096.0 / self.GEAR_RATIO
        angle_friendly = (angle_dxl - self.ZERO_OFFSET) * friendly_per_dxl
        return angle_friendly

    def _deg_to_dxl(self, angle_friendly):
        # Dynamixel joint 0 to 360 deg is 0 to 4096
        friendly_per_dxl = 360.0 / 4096.0 / self.GEAR_RATIO
        angle_dxl = int(angle_friendly / friendly_per_dxl + 0.5) + \
                    self.ZERO_OFFSET
        return angle_dxl
    
    def _set_profile(self, dur_ms):
        # Change profile vel and acc of Dynamixels.
        # Assumes that the motors are set to "Time-based Profile" (ADDR 10)
        for joint in self.JOINTS:
            self.packetHandler.write4ByteTxRx(self.portHandler, joint, 
                                        self.ADDR_PROFILE_VELOCITY, dur_ms)
            self.packetHandler.write4ByteTxRx(self.portHandler, joint, 
                                        self.ADDR_PROFILE_ACCELERATION, 
                                        int(dur_ms/3))
        self.prev_profile = dur_ms
        return
    
    def param_goal_position(self, goal_pos):
        # Allocate goal position value into byte array
        # Split a 32-bit number into 4 8-bit numbers for packet transmission
        return [DXL_LOBYTE(DXL_LOWORD(goal_pos)), 
                DXL_HIBYTE(DXL_LOWORD(goal_pos)),
                DXL_LOBYTE(DXL_HIWORD(goal_pos)), 
                DXL_HIBYTE(DXL_HIWORD(goal_pos))]