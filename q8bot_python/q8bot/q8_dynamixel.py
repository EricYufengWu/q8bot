from dynamixel_sdk import *

DEFAULT_JOINTLIST = [i + 11 for i in range(8)]

class q8_dynamixel:
    def __init__(self, port, joint_list = DEFAULT_JOINTLIST, baud = 1000000):
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
        self.ADDR_D_GAIN = 80
        self.ADDR_P_GAIN = 84
        self.ADDR_PROFILE_ACCELERATION = 108
        self.ADDR_PROFILE_VELOCITY = 112
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_PRESENT_CURRENT = 126
        self.ADDR_PRESENT_POSITION = 132
        self.ADDR_PRESENT_INPUT_VOLTAGE = 144
        self.ADDR_PRESENT_TEMPERATURE = 146
        
        # Initialize PortHandler and PacketHandler instance
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)
        self.groupSyncRead = GroupSyncRead(self.portHandler, 
                                           self.packetHandler, self.ADDR_PRESENT_CURRENT, 10)
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, 
                                           self.packetHandler, self.ADDR_TORQUE_ENABLE, 1)
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
    
    def syncwrite(self, cmd):
        for j in range(len(self.JOINTS)):
            result = self.groupSyncWrite.addParam(self.JOINTS[j], [DXL_LOBYTE(cmd)])
            if not result:
                print('failed to add syncwrite param')
                return False
        self.groupSyncWrite.txPacket()
        self.groupSyncWrite.clearParam()
        return True

    def move_all(self, joints_pos, dur = 0):
        # Expects 8 positions in deg. For example: [0, 90, 0, 90, 0, 90, 0, 90]
        try:
            if dur != self.prev_profile:
                self._set_profile(dur)
            for j in range(len(self.JOINTS)):
                self.groupBulkWrite.addParam(
                    self.JOINTS[j], 
                    self.ADDR_GOAL_POSITION, 
                    self.BYTE_LEN, 
                    self.param_goal_position(self._deg_to_dxl(joints_pos[j])))
            self.groupBulkWrite.txPacket()
            self.groupBulkWrite.clearParam()
        except:
            return False
        return True
    
    def move_mirror(self, joint_pos, dur = 0):
        # Expects a pair of pos for one leg, which will be mirrored 4times.
        mirrored_pos = []
        for i in range(4):
            mirrored_pos.append(joint_pos[0])
            mirrored_pos.append(joint_pos[1])
        return self.move_all(mirrored_pos, dur)
    
    # Reads position and current only.
    def syncread(self):
        try:
            dxl_comm_result = self.groupSyncRead.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            pos_raw = []
            cur_list = []
            for joint in self.JOINTS:
                cur_list.append(self.groupSyncRead.getData(joint, self.ADDR_PRESENT_CURRENT, 2))
                pos_raw.append(self.groupSyncRead.getData(joint, self.ADDR_PRESENT_POSITION, 4))
            value = [pos_raw, cur_list]
        except:
            return [], False
        return value, True
    
    def joint_read4(self, joint, addr):
        value, dxl_comm_result, dxl_error =  self.packetHandler.read4ByteTxRx(
            self.portHandler, 
            joint, 
            addr)
        return value
    
    def joint_read2(self, joint, addr):
        value, dxl_comm_result, dxl_error =  self.packetHandler.read2ByteTxRx(
            self.portHandler, 
            joint, 
            addr)
        return value
    
    def joint_read1(self, joint, addr):
        value, dxl_comm_result, dxl_error =  self.packetHandler.read1ByteTxRx(
            self.portHandler, 
            joint, 
            addr)
        return value
    
    def joint_write4(self, joint, addr, value):
        comm_result, error = self.packetHandler.write4ByteTxRx(self.portHandler, joint, addr, value)
        if comm_result != COMM_SUCCESS:
            print("joint ", joint, ": %s" % self.packetHandler.getTxRxResult(comm_result))
        elif error != 0:
            print("joint ", joint, ": %s" % self.packetHandler.getRxPacketError(error))
        # else:
        #     print("joint write success")
        return

    def check_voltage(self):
        voltage = self.joint_read2(self.JOINTS[0], 
                                   self.ADDR_PRESENT_INPUT_VOLTAGE)
        return voltage / 10
    
    #-------------------#
    # Private Functions #
    #-------------------#
    def _start_comm(self):
        # Open port and set baudrate
        for joint in self.JOINTS:
            result = self.groupSyncRead.addParam(joint)
        if self.portHandler.openPort() and \
        self.portHandler.setBaudRate(self.BAUDRATE) and result:
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
    