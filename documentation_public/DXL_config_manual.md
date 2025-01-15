# Manual Dynamixel Motor Configuration

## Identify the front of the robot:
The "Front" silkscreen marking on the front side of the PCB is the front of the actual robot. When the robot is pointing forward, the Seeed XIAO board and most of the electronics are facing left.

![Q8bot](dxl_manual_setup.png)

## Robot joint assignment
**Note: J1-J8 markings on the PCB are for the PCB only. For the remaining of this doc, please follow the robot image above for actual joint assignment.**

Using Dynamixel Wizard 2.0 and an U2D2 hub, configure each motor's ID, baudrate, and other parameters before mounting them to the robot.

J1 (front left vertical, FLV): <br>
ID = 11<br>
Baud Rate = 1000000<br>
Drive Mode = Time-based Profile<br>
Operating Mode = Extended Position control<br>
Homing Offset = 2048<br>
**(Only applies when mounting the legs)** When linkage points forward, position should read 360.<br>

J2 (front left horizontal, FLH):<br>
ID = 12<br>
Baud Rate = 1000000<br>
Drive Mode = Time-based Profile<br>
Operating Mode = Extended Position control<br>
Homing Offset = 4096<br>
When linkage points downward, position should read 450.<br>

J3 (front right vertical, FRV): <br>
ID = 13<br>
Baud Rate = 1000000<br>
Drive Mode = Time-based Profile, Reverse mode<br>
Operating Mode = Extended Position control<br>
Homing Offset = 2048<br>
When linkage points forward, position should read 360.<br>

J4 (front right horizontal, FRH):<br>
ID = 14<br>
Baud Rate = 1000000<br>
Drive Mode = Time-based Profile, Reverse mode<br>
Operating Mode = Extended Position control<br>
Homing Offset = 4096<br>
When linkage points downward, position should read 450.<br>

J5 (back left horizontal, BLH): <br>
ID = 15<br>
Baud Rate = 1000000<br>
Drive Mode = Time-based Profile<br>
Operating Mode = Extended Position control<br>
Homing Offset = 2048<br>
When linkage points forward, position should read 360.<br>

J6 (back left vertical, BLV):<br>
ID = 16<br>
Baud Rate = 1000000<br>
Drive Mode = Time-based Profile<br>
Operating Mode = Extended Position control<br>
Homing Offset = 4096<br>
When linkage points downward, position should read 450.<br>

J7 (bask right horizontal, BRH): <br>
ID = 17<br>
Baud Rate = 1000000<br>
Drive Mode = Time-based Profile, Reverse mode<br>
Operating Mode = Extended Position control<br>
Homing Offset = 2048<br>
When linkage points forward, position should read 360.<br>

J8 (back right vertical, BRV):<br>
ID = 18<br>
Baud Rate = 1000000<br>
Drive Mode = Time-based Profile, Reverse mode<br>
Operating Mode = Extended Position control<br>
Homing Offset = 4096<br>
When linkage points downward, position should read 450.<br>