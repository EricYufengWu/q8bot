from dynamixel_sdk import *
import serial

DEFAULT_JOINTLIST = [i + 11 for i in range(8)]

class q8_espnow:
    def __init__(self, port, joint_list = DEFAULT_JOINTLIST, baud = 115200):
        self.DEVICENAME = port
        self.BAUDRATE = baud
        self.JOINTS = joint_list
        self.prev_pos = [90 for i in range(8)]
        self.prev_profile = 0
        self.torque_on = False

        # Initialize serial communication with ESP32-C3
        self.serialHandler = serial.Serial(self.DEVICENAME, self.BAUDRATE)

        # # Dynamixel constants
        # self.PROTOCOL_VERSION = 2.0
        # self.BYTE_LEN = 4
        # self.TORQUE_ENABLE = 1
        # self.TORQUE_DISABLE = 0
        # self.DXL_MOVING_STATUS_THRESHOLD = 20
        # self.ZERO_OFFSET = 4096
        # self.GEAR_RATIO = 1

        # # Dynamixsl X Series registers
        # self.ADDR_TORQUE_ENABLE = 64
        # self.ADDR_PROFILE_ACCELERATION = 108
        # self.ADDR_PROFILE_VELOCITY = 112
        # self.ADDR_GOAL_POSITION = 116
        # self.ADDR_PRESENT_POSITION = 132
        # self.ADDR_PRESENT_INPUT_VOLTAGE = 144
        
        # # Initialize PortHandler and PacketHandler instance
        # self.portHandler = PortHandler(self.DEVICENAME)
        # self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)
        # self.groupBulkRead = GroupBulkRead(self.portHandler, 
        #                                    self.packetHandler)
        # self.groupBulkWrite = GroupBulkWrite(self.portHandler, 
        #                                      self.packetHandler)
        # self._start_comm()

    def enable_torque(self):
        self.serialHandler.write("0,0,0,0,0,0,0,0,0,1;".encode())
        self.torque_on = True
        return True
    
    def disable_torque(self):
        self.serialHandler.write("0,0,0,0,0,0,0,0,0,0;".encode())
        self.torque_on = False
        return True

    def move_all(self, joints_pos, dur = 0):
        # Expects 8 positions in deg. For example: [0, 90, 0, 90, 0, 90, 0, 90]
        try:
            cmd = ",".join(map(str, joints_pos)) + "," + f"{dur}," + f"{int(self.torque_on)};" 
            # cmd = f"{int(self.torque_on)}," + f"{dur}," + ",".join(map(str, joints_pos)) + ";"
            print(cmd)
            self.serialHandler.write(cmd.encode())
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
    
    def bulkread(self, addr, len = 4):
        value = [0 for i in range(8)]
        return value, True
    
    def joint_read4(self, joint, addr):
        value = 10
        return value
    
    def joint_read2(self, joint, addr):
        value = 10
        return value

    def check_voltage(self):
        voltage = 3.7
        return voltage
    
    #-------------------#
    # Private Functions #
    #-------------------#

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
        return