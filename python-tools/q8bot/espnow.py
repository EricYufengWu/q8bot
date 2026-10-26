
'''
Written by yufeng.wu0902@gmail.com

Python class that passes arguments to connected ESP32C3 controller, which
then sends commands wirelessly to the robot via ESPNow.
'''

import serial

DEFAULT_JOINTLIST = [i + 11 for i in range(8)]

# Robot replies arrive from the controller as one line of 100 numbers. These
# tags in the first number mark the replies used for leg detection; they sit
# above anything a battery reading or a recorded sample can produce.
TAG_TELEMETRY = 61001
TAG_PROBE = 61002
TAG_CAP = 61003
TAGS = (TAG_TELEMETRY, TAG_PROBE, TAG_CAP)

# Probe outcomes, matching firmware/q8bot_robot/include/legProbe.h.
REASONS = {0: 'none', 1: 'contact', 2: 'overcurrent', 3: 'bound', 4: 'held'}
STATUSES = {
    0: 'ok', 1: 'robot busy with another probe', 2: 'request refused',
    3: 'a servo still has torque on', 4: 'a servo reports a fault or does not answer',
    5: 'servo setup failed', 6: 'a servo raised a hardware fault',
    7: 'a servo stopped answering', 8: 'a held joint was pushed out of place',
    9: 'aborted', 10: 'sweep timed out',
    11: 'torque-off could not be confirmed on every servo',
}
PROBE_STATUS_OK = 0
PROBE_RESTORE_FAILED = 11


def ticks_to_deg(ticks):
    # Firmware convention: 0 deg = 4096 ticks, 4096 ticks per turn.
    return (ticks - 4096) * 360.0 / 4096.0


def parse_values(line):
    """A reply line as a list of ints, or None if it is not one."""
    if isinstance(line, bytes):
        line = line.decode('utf-8', errors='replace')
    parts = line.strip().split()
    if not parts or not all(p.isdigit() for p in parts):
        return None
    return [int(p) for p in parts]


def decode_reply(values):
    """Decode a tagged reply, or return None for anything else."""
    if not values or values[0] not in TAGS:
        return None
    if values[0] == TAG_TELEMETRY:
        servos = {}
        for i in range(min(values[1], 9)):
            flags, position, current, error = values[2 + i * 4: 6 + i * 4]
            servos[11 + i] = {
                'responding': bool(flags & 1), 'torque': bool(flags & 2),
                'fault': bool(flags & 4), 'ticks': position - 32768,
                'deg': ticks_to_deg(position - 32768), 'current': current - 32768,
                'error': error,
            }
        return {'kind': 'telemetry', 'servos': servos}
    if values[0] == TAG_PROBE:
        joints = []
        # Slots 0-7 are the leg joints (IDs 11-18); slot 8 is the head (ID 19).
        for i in range(9):
            flags, reason, start, stop, peak, deflection = values[2 + i * 6: 8 + i * 6]
            joints.append({
                'involved': bool(flags & 1), 'swept': bool(flags & 2),
                'reason': REASONS.get(reason, 'unknown'),
                'start': ticks_to_deg(start - 32768) if flags & 1 else None,
                'stop': ticks_to_deg(stop - 32768) if flags & 1 else None,
                'peak_current': peak, 'deflection': deflection * 360.0 / 4096.0,
            })
        return {'kind': 'probe', 'status': values[1],
                'status_text': STATUSES.get(values[1], 'unknown status'), 'joints': joints}
    return {'kind': 'cap', 'pwm': values[1], 'confirmed': values[2]}

class q8_espnow:
    def __init__(self, port, joint_list = DEFAULT_JOINTLIST, baud = 115200):
        self.DEVICENAME = port
        self.BAUDRATE = baud
        self.JOINTS = joint_list
        self.prev_pos = [90 for i in range(8)]
        self.prev_profile = 0
        self.torque_on = False
        # Whole turns added to each joint's command, so commands land on the
        # turn the servo's reading is on (see set_turn_offsets).
        self.turn_offsets = [0] * 8

        # Initialize serial communication with ESP32-C3
        self.serialHandler = serial.Serial(self.DEVICENAME, self.BAUDRATE)

    def enable_torque(self):
        self.serialHandler.write("0,0,0,0,0,0,0,0,0,0,1;".encode())
        self.torque_on = True
        return True
    
    def disable_torque(self):
        self.serialHandler.write("0,0,0,0,0,0,0,0,0,0,0;".encode())
        self.torque_on = False
        return True
    
    def check_battery(self):
        self.serialHandler.write("0,0,0,0,0,0,0,0,1,0,0;".encode())
        return True
    
    def record_data(self):
        self.serialHandler.write("0,0,0,0,0,0,0,0,2,0,0;".encode())
        return True
    
    def finish_recording(self):
        self.serialHandler.write("0,0,0,0,0,0,0,0,3,0,1;".encode())
        return True
    
    def send_jump(self):
        self.serialHandler.write("0,0,0,0,0,0,0,0,4,0,0;".encode())
        return True

    def send_recover(self):
        # Clears latched servo faults and re-enables torque on the robot. The
        # local flag must follow, or the next move_all would carry torque=0 and
        # the firmware would read that as a request to switch torque back off.
        self.serialHandler.write("0,0,0,0,0,0,0,0,5,0,1;".encode())
        self.torque_on = True
        return True

    def request_telemetry(self):
        """Ask for a read-only snapshot of all nine servos (reply TAG_TELEMETRY)."""
        self.serialHandler.write(f"0,0,0,0,0,0,0,0,6,0,{int(self.torque_on)};".encode())
        return True

    def start_probe(self, excursions, pwm):
        """Sweep joints by signed bounds in degrees, 0 = not swept (reply TAG_PROBE).

        Torque must be off. The robot holds each swept joint's partner, stops
        at contact or at the bound, returns the joints and releases them.
        """
        fields = ",".join(f"{float(e):.1f}" for e in excursions)
        self.serialHandler.write(f"{fields},7,{int(pwm)},0;".encode())
        return True

    def start_head_probe(self, bound, pwm):
        """Sweep the head (ID 19) by a signed bound in degrees (reply TAG_PROBE, slot 8)."""
        self.serialHandler.write(f"{float(bound):.1f},0,0,0,0,0,0,0,11,{int(pwm)},0;".encode())
        return True

    def move_head(self, deg, ms):
        """Head yaw in degrees from its centre, reached over ``ms``; no reply.

        The robot clamps to +-90 deg and caps the head's output at ~40 %; the
        caller keeps the angle inside the measured collision limits.
        """
        self.serialHandler.write(f"{float(deg):.1f},0,0,0,0,0,0,0,10,{int(ms)},{int(self.torque_on)};".encode())
        return True

    def abort_probe(self):
        self.serialHandler.write("0,0,0,0,0,0,0,0,8,0,0;".encode())
        return True

    def set_output_cap(self, pwm):
        """Goal PWM cap for normal motion on IDs 11-18, 885 = full (reply TAG_CAP)."""
        self.serialHandler.write(f"{int(pwm)},0,0,0,0,0,0,0,9,0,{int(self.torque_on)};".encode())
        return True

    def set_turn_offsets(self, present_deg, model_deg):
        """Pick, per joint, the whole turn that puts model angles on the servo's turn.

        An extended-position servo starts each power-up with its reading in one
        fixed turn. A leg that sat past that turn's edge at switch-on reads 360
        degrees away from the angle the kinematic model uses for the same pose;
        commanding the model angle would turn it a full revolution. present_deg
        is the reading, model_deg the same pose as the model expresses it.
        """
        self.turn_offsets = [round((p - m) / 360.0) for p, m in zip(present_deg, model_deg)]
        return list(self.turn_offsets)

    def arm(self, model_positions):
        """Torque on, holding the given pose, with nothing moving.

        With torque off, goals are written first (the firmware writes goals on
        a packet that leaves torque unchanged), then torque comes on. With
        torque already on, the goals are simply updated: dropping torque to
        re-arm would let a standing robot fall.
        """
        if self.torque_on:
            return self.move_all(model_positions, 0, False)
        self.move_all(model_positions, 0, False)
        return self.enable_torque()

    def move_all(self, joints_pos, dur = 0, record = True):
        # Expects 8 positions in deg. For example: [0, 90, 0, 90, 0, 90, 0, 90]
        try:
            joints_pos = [p + 360 * k for p, k in zip(joints_pos, self.turn_offsets)]
            # If record is true, the 9th element is set to value 2. Else 0.
            cmd = ",".join(map(str, joints_pos)) + f",{record*2}," + f"{dur}," + f"{int(self.torque_on)};" 
            # print(cmd)
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
        return