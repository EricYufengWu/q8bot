# Software Setup Instructions

[Sourcing Components](sourcing_components.md)

[Assembling the Robot](robot_assembly.md)

[**Software Setup**]()

[Back to Project Page](https://github.com/EricYufengWu/q8bot)

## Before You Begin 📋

- These instructions assume that you have **fully assembled the robot** by following the previous assembly steps.
- Ensure that the batteries are **charged to a reasonable level**.
- Have both the Q8bot (subsequently called **"the robot"**) and the separate Seeed Studio XIAO ESP32C3 for your laptop (subsequently called **"the controller"**).

## Robot Firmware Setup 📟

#### 1. Download required software
Download [**Visual Studio Code**](https://code.visualstudio.com/download) to your laptop. Once VSCode installed, follow [**this instruction page**](https://platformio.org/install/ide?install=vscode) to install the PlatformIO IDE extension.

Download or clone the [**Q8bot repository**](https://github.com/EricYufengWu/q8bot). 

<p align="center">
    <img src="sw_repo.png" width="75%">
</p>

#### 2. Open the PlatformIO project and prepare the robot
Open VSCode, and go to File -> Open Folder. Open `firmware/q8bot_robot`. If you are opening this PlatformIO project for the first time, it might take a minute or so to install all the dependencies.

With batteries installed on your robot and its power button switched on, connect your Q8bot PCB to your laptop using a USB-C cable.

#### 3. Configure the build environment

In your PlatformIO window, first click the `env:robot_auto (q8bot_robot)` selector at the **bottom** of the window, then select `env:robot_permanent` from the drop-down menu.

<p align="center">
    <img src="sw_firmware_env.png" width="75%">
</p>

You may also change the COM port to match the actual port your robot is plugged into.  
Leaving it set to **“Auto”** works most of the time, as long as only one device is connected.

<p align="center">
    <img src="sw_firmware_com.png" width="75%">
</p>

#### 4. Upload firmware to the robot

With your robot plugged in, click the **arrow button** to start the compiling and uploading process.

<p align="center">
    <img src="sw_firmware_upload.png" width="75%">
</p>

## Python Setup 💻

Install python locally on your computer if you have not already. The simplest way is through the [official website](https://www.python.org/downloads/) (the latest version will do). **Make sure to check the "add python.exe to PATH" option.**
<p align="center">
    <img src="sw_python.jpg" width="60%">
</p>

It's is best to set up a virtual environment to prevent dependency conflicts between different projects.

In VS Code, go to **File → New Window**, then **File → Open Folder**.  
Open the `/python-tools` folder inside the main Q8bot repository.

Under **TERMINAL**, run the following command to create a virtual environment for the project:

    python -m venv venv

<p align="center">
    <img src="sw_python_cmd.png" width="75%">
</p>

Wait for the command to finish, then activate the virtual environment:

    .\venv\Scripts\activate

On macOS/Linus, run this instead:

    source venv/bin/activate

In the same directory, run the following:

    pip install --upgrade --force-reinstall -r requirements.txt

This will force install necessary dependencies in your venv only (there aren't a lot so you may have already had all libraries installed). For some systems, the above command might not work, in which case you may manually use `pip` to install the libraries.

## Running the Robot 🚀
Attach the batteries to the robot (double-check polarity!). Power on the robot with the onboard slide switch and you should see the onboard LED light up.

Plug in the controller board to your laptop/PC.

Navigate to `/python-tools/q8bot` folder and run:

    python operate.py

If everything works, a Pygame window should open with two partitions:

- Top panel: live debug info
- Bottom panel: a static image with keyboard controls

Make sure the Pygame window has focus (click on it). On launch, the robot will move its joints to their initial (home) positions. 

A quick note on **[R]**: This is useful when you notice the robot's movement suddenly becomes super slow (temporary fix to a know bug, possible EE-related). If you have to frequently do this, the robot's battery may be low.

<p align="center">
    <img src="sw_default_screen.jpg" alt="High level flowchart" width="60%">
</p>

If a joystick (USB or Bluetooth) is connected, a different Pygame screen appears showing button mappings and live input status.

<p align="center">
    <img src="sw_joystick_screen.jpg" alt="High level flowchart" width="60%">
</p>

Have fun!

## Common Issues

**Q: Pygame window launches but the robot is not moving with my command**

A: Try restarting the robot, re-inserting the dongle, and rerun the python script. If you are still having trouble, follow the steps below to initiate a "force repair":
- Connect both the robot and the dongle to your laptop.
- Open two separate serial monitors (you can use the "Serial Monitor" plugin from VSCode). 
- In both windows, type 'd' to turn on debugging for both devices.
- Type 'p' to each device to turn on pairing mode. You should see successfull pairing message from their serial monitors.

---

<br><br>

# Appendix

## Complete Firmware Uploading Instructions

Different firmware need to be uploaded to Q8bot (subsequently called **"robot"**) and the separate Seeed Studio XIAO ESP32C3 connected to your host laptop (subsequently called **"controller"**). 

If you check the latest project release, you will notice that we have two available sets of firmware: `firmware-perm-espnow-x.bin` and `firmware-auto-espnow-x.bin`. The following table summarizes their differences:

|                           | `firmware-perm-espnow-x.bin` | `firmware-auto-espnow-x.bin` |
|---------------------------|----------------------------|----------------------------|
| First time pairing        | Controller automatically pairs with the first robot it sees. Both devices store their counterpart's MAC address in the NVS memory (stays there after reset)| Same as `firmware-perm-espnow-x.bin` |
| How to un-pair and repair | Manually send 'p' via serial monitor to both the controller and the robot| Both devices will automatically attempt to find new devices and re-pair after a 5-second timeout. |
| Use this if...            | You want multiple robot-controller pairs to work reliably, such as in a class room setting **(I generally recommend to start with this)** | If you want one controller on multiple robots without manually un-pairing every time (gets messy with 2 or more pairs running at the same time) |

The microcontroller part of the code is developed in [PlatformIO](https://platformio.org/). If you haven't used it before, please refer to their official documentation and tutorials to setup the environment. Someone has also tried converting PlatformIO projects to Arduino IDE script [here](https://runningdeveloper.com/blog/platformio-project-to-arduino-ide/).

Upload `firmware/q8bot_robot` to the Q8bot robot, and upload `firmware/q8bot_controller` to the Q8bot controller. The process will be similar to steps 10 - 13 in [Robot Assembly](robot_assembly.md). Make sure you have selected your desired build (perm vs auto) as shown in the screenshot below.The default is robot_permanent/controller_permanent.

<p align="center">
    <img src="sw_buildver.jpg" width="90%">
</p>

For advanced users, the quickest way to upload the firmware is by using [esptool](https://docs.espressif.com/projects/esptool/en/latest/esp32/) from Espressif. Replace `COM9` with the actual port that your device shows up as.  

    python -m esptool --port COM9 write_flash 0x10000 .\firmware-perm-espnow-robot.bin

    python -m esptool --port COM9 write_flash 0x10000 .\firmware-perm-espnow-controller.bin

## [Experimental] Running Q8bot Using the Standalone Executable
In the latest release, you will find a `q8bot_operate.exe` file, which allows you to skip all the Python setup and run Q8bot directly. Simply download the executable, plug in your controller to the laptop, turn on your robot, and double-click the executable. Wait until you see a blank screen named "pygame" appear, and you will be able to control the robot using the same keyboard commands listed later in this document.

Please note that this method has not been thoroughly tested and may contain more bugs. It also only works if you have a Windows machine. For the alternative (classic) method, please proceed to setting up python in your host laptop to run the program. This would be a better learning opportunity, too!


## Software Overview

Currently, all computation regarding gait generation and FK/IK happens on the laptop. The laptop talks to the robot remotely via another Seeed Studio XIAO ESP32C3, sending raw joint angles as rapidly as possible using the ESPNow protocol. In the future, the hope is to move the gait generation code to the robot's onboard ESP32 for improved control.

<p align="center">
    <img src="High_Level_Flowchart.jpg" alt="High level flowchart" width="60%">
</p>

Here's a rough overview to the logic behind the python script (might not be accurate as I keep adding features)

![alt text](Python_Flowchart.jpg)