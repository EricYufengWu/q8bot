# Software Setup Instructions

[Sourcing Components](sourcing_components.md)

[Assembling the Robot](robot_assembly.md)

[**Software Setup**]()

[Back to Project Page](https://github.com/EricYufengWu/q8bot)

## Software Overview

Please excuse my messy code as I am a mechanical engineer by training :D

Currently, all computation regarding gait generation and FK/IK happens on the laptop. The laptop talks to the robot remotely via another Seeed Studio XIAO ESP32C3, sending raw joint angles as rapidly as possible using the ESPNow protocol. In the future, the hope is to move the gait generation code to the robot's onboard ESP32 for improved control.

<p align="center">
    <img src="High_Level_Flowchart.jpg" alt="High level flowchart" width="60%">
</p>

## Seeed Studio XIAO MCU Setup (Robot + Controller)

The microcontroller part of the code is developed in [PlatformIO](https://platformio.org/). If you haven't used it before, please refer to their official documentation and tutorials to setup the environment. Someone has also tried converting PlatformIO projects to Arduino IDE script [here](https://runningdeveloper.com/blog/platformio-project-to-arduino-ide/).

Different firmware need to be uploaded to Q8bot ("robot") and the separate Seeed Studio XIAO ESP32C3 ("controller"): `firmware/q8bot_robot` and `firmware/q8bot_controller`. Before they are ready to upload, you need to:
- Navigate to `firmware/q8bot_robot/main.cpp` and update the MAC address to match your controller
- Navigate to `firmware/q8bot_controller/main.cpp` and update the MAC address to match your robot.
- A quick way to find the MAC address of your device is by selecting the specific COM port in PlatformIO.
<p align="center">
    <img src="sw_MAC.png" width="90%">
</p>

Upload `firmware/q8bot_robot` to the Q8bot PCB, and upload `firmware/q8bot_controller` to the controller ESP32C3. The process will be similar to steps 10 - 13 in [Robot Assembly](robot_assembly.md), 

## Python Setup
Install python locally on your computer if you have not already. The simplest way is through the [official website](https://www.python.org/downloads/) (the latest version will do). **Make sure to check the "add python.exe to PATH" option.**
<p align="center">
    <img src="sw_python.png" width="60%">
</p>

It's is best to set up a virtual environment to prevent dependency conflicts between different projects.

Using a terminal of your choise, navigate to the `/python-tools` folder and run the following to create a virtual environment for your project:

    python -m venv venv

In the same directory, activate the virtual environment.

    .\venv\Scripts\activate

In the same directory, run the following:

    pip install --upgrade --force-reinstall -r requirements.txt

This will force install necessary dependencies in your venv only (there aren't alot so you mey have already had all libraries installed).

## Running the Robot
Attach the batteries to the robot (double-check polarity!). Power on the robot with the onboard slide switch and you should see the onboard LED light up.

Plug in the controller board to your laptop/PC.

Navigate to `/python-tools/q8bot` folder and run:

    python operate.py

If everything works, you should see a small pygame window pop up and the robot move its joints to their initial location. Robot keyboard control instructions are as follows:
- WASD for robot movement. Q and E are used to partial turning in amber gait.
- G two cycle between different gaits.
- J for jumping.
- R for resetting stance, size, etc.
- The terminal/console will output useful information depending on your input.

Have fun!

## Appendix

Here's a rough overview to the logic behind the python script (might not be accurate as I keep adding features)

![alt text](Python_Flowchart.jpg)
