
'''
Written by yufeng.wu0902@gmail.com

Python class that passes arguments to connected ESP32C3 controller, which
then sends commands wirelessly to the robot via ESPNow.
'''

from enum import Enum
import serial
import struct

from gait_manager import GAITS, GAIT_DIRECTIONS

DEFAULT_JOINTLIST = [i + 11 for i in range(8)]

# bytes of payload should be placed at the end of this
# see firmware/q8bot_robot/include/communications.h
# cmd_type, number of payload elements, payload
MESSAGE_FORMAT = "<BB"

class Commands(Enum):
    MOVE = 0
    BATTERY = 1
    RECORD = 2
    SEND_RECORDED = 3
    JUMP = 4
    SET_TORQUE = 5
    SET_PROFILE = 6
    SET_GAIT = 7

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

    def enable_torque(self):
        self.serialHandler.write(struct.pack(MESSAGE_FORMAT, Commands.SET_TORQUE.value, 1) + (1).to_bytes(4, 'little'))
        self.torque_on = True
        return True
    
    def disable_torque(self):
        self.serialHandler.write(struct.pack(MESSAGE_FORMAT, Commands.SET_TORQUE.value, 1) + (0).to_bytes(4, 'little'))
        self.torque_on = False
        return True
    
    def check_battery(self):
        self.serialHandler.write(struct.pack(MESSAGE_FORMAT, Commands.BATTERY.value, 0))
        return True
    
    def record_data(self):
        self.serialHandler.write(struct.pack(MESSAGE_FORMAT, Commands.RECORD.value, 0))
        return True
    
    def finish_recording(self):
        self.serialHandler.write(struct.pack(MESSAGE_FORMAT, Commands.SEND_RECORDED.value, 0))
        return True
    
    def send_jump(self):
        self.serialHandler.write(struct.pack(MESSAGE_FORMAT, Commands.JUMP.value, 0))
        return True
    
    def start_gait(self, gait_name: str, gait_direction: str):
        self._set_profile(0)
        
        gait_id = GAIT_DIRECTIONS.index(gait_direction) * len(list(GAITS.keys())) + list(GAITS.keys()).index(gait_name)

        self.serialHandler.write(struct.pack(MESSAGE_FORMAT, Commands.SET_GAIT.value, 1) + struct.pack("<i", gait_id))
        return True
    
    def stop_gait(self):
        self.serialHandler.write(struct.pack(MESSAGE_FORMAT, Commands.SET_GAIT.value, 0))
        return True

    def move_all(self, joints_pos, dur = 0, record = True):
        # Expects 8 positions in deg. For example: [0, 90, 0, 90, 0, 90, 0, 90]
        try:
            values = []

            values += map(float, joints_pos)
            assert len(values) == 8

            self._set_profile(dur)

            self.serialHandler.write(struct.pack(MESSAGE_FORMAT, Commands.MOVE.value, 8) + struct.pack("<8f", *values))

            if record:
                self.record_data()
        except:
            return False
        return True
    
    def move_mirror(self, joint_pos, dur = 0):
        # Expects a pair of pos for one leg, which will be mirrored 4times.
        mirrored_pos = []
        for i in range(4):
            mirrored_pos.append(joint_pos[0])
            mirrored_pos.append(joint_pos[1])
        return self.move_all(mirrored_pos, dur, False)
    
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

    def dxl2deg(self, angle_dxl):
        # Dynamixel joint 0 to 360 deg is 0 to 4096
        friendly_per_dxl = 360.0 / 4096.0 / self.GEAR_RATIO
        angle_friendly = (angle_dxl - self.ZERO_OFFSET) * friendly_per_dxl
        return angle_friendly

    def deg2dxl(self, angle_friendly):
        # Dynamixel joint 0 to 360 deg is 0 to 4096
        friendly_per_dxl = 360.0 / 4096.0 / self.GEAR_RATIO
        angle_dxl = int(angle_friendly / friendly_per_dxl + 0.5) + \
                    self.ZERO_OFFSET
        return angle_dxl
    
    #-------------------#
    # Private Functions #
    #-------------------#
    
    def _set_profile(self, dur_ms):
        self.serialHandler.write(struct.pack(MESSAGE_FORMAT, Commands.SET_PROFILE.value, 1) + struct.pack("<i", dur_ms))
        return