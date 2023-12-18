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

JUMP_LOW = [11, 169] 
JUMP_HIGH = [95, 85]
JUMP_REST = [30, 150]

# Main code
def main():
    leg = q8_dynamixel('COM3')
    leg.enable_torque()

    # Main Loop
    while(1):
        print("Press any key to continue (or press ESC to quit)")
        if getch() == '\x1b':    #esc key
            break

        time.sleep(2)
        leg.move_leg(JUMP_LOW, 500)
        time.sleep(0.5)
        leg.move_leg(JUMP_HIGH, 0)
        time.sleep(0.08)
        leg.move_leg(JUMP_REST, 0)
        time.sleep(0.5)
    
    leg.disable_torque()

if __name__ == "__main__":
    main()