# Note: this program assumes the DXL motors are set to "Time-based Profile" 
# in the Drive Mode register (ADDR 10)!

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

from q8_dynamixel import *

# jump_start   = [12, 168] 
# jump_start   = [-25, 205] 
jump_start   = [-50, 230] 
jump_end     = [90, 90]
jump_rest    = [90, 90]
# jump_rest    = [0, 180]

# Main code
def main():
    leg = q8_dynamixel('COM5', joint_list = [11, 12])
    leg.enable_torque()

    # Main Loop
    while(1):
        print("Press any key to continue (or press ESC to quit)")
        if getch() == '\x1b':    #esc key
            break

        time.sleep(2)
        leg.move_all(jump_start, 500)
        time.sleep(0.8)
        leg.move_all(jump_end, 0)
        time.sleep(0.08)
        leg.move_all(jump_rest, 0)
        time.sleep(0.5)
    
    # leg.disable_torque()

if __name__ == "__main__":
    main()