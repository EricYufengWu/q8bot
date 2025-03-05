# Software Setup Instructions

[Sourcing Components](sourcing_components.md)

[Assembling the Robot](robot_assembly.md)

[**Software Setup**]()

[Back to Project Page](https://github.com/EricYufengWu/q8bot)

## Software Setup

Please excuse my messy code as I am a mechanical engineer by training :D

Currently, all computation regarding gait generation and FK/IK happens on the laptop. The laptop talks to the robot remotely via another Seeed Studio XIAO ESP32C3, sending raw joint angles as rapidly as possible using the ESPNow protocol. In the future, the hope is to move the gait generation code to the robot's onboard ESP32 for improved control.

<p align="center">
    <img src="High_Level_Flowchart.jpg" alt="High level flowchart" width="60%">
</p>

### Seeed Studio XIAO MCU Setup (Robot + Controller)

The microcontroller part of the code is developed in [PlatformIO](https://platformio.org/). If you haven't used it before, please refer to their official documentation and tutorials to setup the environment. Someone has also tried converting PlatformIO projects to Arduino IDE script [here](https://runningdeveloper.com/blog/platformio-project-to-arduino-ide/).

Open the folder "q8bot_robot" with PlatformIO and upload it to Q8bot's XIAO board.

Currently, Q8bot uses another ESP32C3 connected to the host PC/laptop. Whether you are using the additional XIAO board as is or in its [dongle form](https://github.com/EricYufengWu/ESPNowDongle), you need to open the folder "q8bot_controller" with PlatformIO and upload it to the controller board.

The MAC address in the controller's code need to be modified to match the address of your robot board.

### Python Setup
Navigate to the `/q8bot_python` folder and run:

    pip install -r requirements.txt

This will install necessary dependencies (there aren't alot so you mey have already had all libraries installed).

Modify the COM port value in `/q8bot_python/q8bot/q8_operate.py` to match the COM port of your controller board.

    PORT = 'COM6' 

### Running the Robot
Attach the batteries to the robot (double-check polarity!). Power on the robot with the onboard slide switch and you should see the onboard LED light up.

Plug in the controller board to your laptop/PC.

Navigate to `/q8bot_python/q8bot` folder and run:

    python q8_operate.py

If everything works, you should see a small pygame window pop up and the robot move its joints to their initial location. Robot keyboard control instructions are as follows:
- WASD for robot movement. Q and E are used to partial turning in amber gait.
- G two cycle between different gaits.
- J for jumping.
- Keyboard up and down for adjusting robot stance.
- "+" and "-" for adjusting stride size.
- R for resetting stance, size, etc.
- (More functions comming soon).

## Appendix

Here's a rough overview to the logic behind the python script (might not be accurate as I keep adding features)

![alt text](Python_Flowchart.jpg)
