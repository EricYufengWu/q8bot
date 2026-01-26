# Robot Assembly Instructions 🤖

[Sourcing Components](sourcing_components.md)

[**Assembling the Robot**]()

[Software Setup](software_setup.md)

[Back to Project Page](https://github.com/EricYufengWu/q8bot)

---

## Before You Begin 📋

- These instructions assume that you have either:  
  (a) purchased a **fully assembled Q8bot PCB** from PCBWay, or  
  (b) reflow-soldered the PCB yourself following the instructions in the [`/PCBA`](../PCBA/) folder.

- If you get stuck or find any part of the instructions confusing, please refer to the **latest CAD model**.  
  If things are still unclear, feel free to contact the author for clarification.

- If you are assembling the robot with **brand new DYNAMIXEL motors**, please have a **laptop ready** to upload a simple firmware for initial motor configuration.

## Assembly Time! 🔧

### 1. Gather your components

From left to right, top to bottom:

- 2x Lithium-ion batteries
- 3D-printed parts
- 12x 692ZZ ball bearings
- Battery clips
- A Separate Seeed Studio XIAO ESP32C3 (connect to PC for wireless control)
- Fully-assembled Q8bot PCB
- 8x brand new DYNAMIXEL XL330-M077-T (which contains the screws needed for assembly)

<p align="center">
    <img src="assem_components.jpg" alt="Components" width="75%">
</p>

Here is another view of the components with all packaging removed.  
Note that we will **only use the 6 mm screws** that come with the DYNAMIXEL motors.

💡 **Tip:** Start charging your batteries now so they are ready later in the assembly process.

<p align="center">
    <img src="assem_nopackage.jpg" alt="Components" width="75%">
</p>

---

### 2. Attach battery clips to the main frames

**Double-check the orientation carefully.**

When holding the frame as shown (with the battery geometry on the left):
- The **negative clip** (with spring) should be on the **left**
- The **positive clip** (flat) should be on the **right**
- The notch should face **inward**

Repeat this step for the second, identical frame.

<p align="center">
    <img src="assem_frame1.jpg" alt="Battery clip orientation" width="75%">
</p>
<p align="center">
    <img src="assem_frame2.jpg" alt="Battery clip installed" width="75%">
</p>

---

### 3. Attach the main frame to the center PCB

Use **8× M2×6 mm self-tapping screws** that came with the DYNAMIXEL motors.

<p align="center">
    <img src="assem_attachframe1.jpg" alt="Attach frame to PCB" width="75%">
</p>
<p align="center">
    <img src="assem_attachframe2.jpg" alt="Frame attached" width="75%">
</p>

---

### 4. Solder battery clips

Use a soldering iron to solder the battery clips to the PCB.

⚠️ **Be careful not to melt the 3D-printed frame parts.**  
Using solder flux can help improve solder flow and reduce heating time.

<p align="center">
    <img src="assem_solder.jpg" alt="Soldering battery clips" width="75%">
</p>

---

### 5. Install ball bearings

Insert the ball bearings into the 3D-printed leg links  
(**12 bearings total**). You may use the back of a screwdriver to gently press the bearings into place.

If a bearing feels loose in the 3D-printed part, a **small amount of super glue** may be applied for reinforcement. ⚠️ Be careful **not to let glue enter the bearing**, as this will prevent it from spinning freely.

<p align="center">
    <img src="assem_bearing.jpg" alt="Installing bearings" width="75%">
</p>

---

### 6. Assemble legs

You will assemble **two mirrored pairs of legs** and set them aside for later. This step has proven to be confusing in the past — take your time, and don’t hesitate to ask questions💡.

Each leg uses **3× M2×6 mm screws**.

Before installing screws, it may help to:
- Lay out the leg parts in the orientation shown below  
- Observe the subtle differences between similar-looking parts  

Refer to the CAD model if anything is unclear.

<p align="center">
    <img src="assem_legs1.jpg" alt="Leg parts orientation" width="75%">
</p>

**Assembly steps (per leg):**
- Position the leg elements as shown and insert the **bottom screw**
- Flip the assembly over and install the remaining **two screws**

<p align="center">
    <img src="assem_legs2.jpg" alt="Leg assembly step 1" width="43.75%">
    <img src="assem_legs3.jpg" alt="Leg assembly step 2" width="46%">
</p>

Temporarily set the assembled legs aside, and let’s return to working on the PCB.

---

### 7. Install the batteries

⚠️ Make sure the negative side goes to the clip with spring, and the positive side goes to the flat clip.

Locate the switch and slide it toward the USB-C port to turn on the power. If your batteries have been installed correctly, you should see the onboard LED of the Seeed Studio XIAO board light up.

**Turn off power for now and proceed to the next step.**

---

### 8. Prepare and open the DYNAMIXEL motors

We will now install the **DYNAMIXEL motors**.

To make the motors easier to handle in the following steps, place a small piece of tape around **each motor**, positioned as shown below. This tape will help keep the **top shell from falling off** once the motor is opened.

Next, **for all 8 motors**:
- Loosen and remove the **four long screws** on the bottom of each motor.
- Carefully detach the **bottom shell**, exposing the motor electronics.
- Keep the screws — you will reuse them later.

<p align="center">
  <img src="assem_dxl1.jpg" alt="detach motor shells" width="45%">
  <img src="assem_dxl2.jpg" alt="detach motor shells" width="45%">
</p>

---

### 9. Identify motor IDs

Each DYNAMIXEL motor has an **ID number label**, from #11 to #18.  
You will also find a **matching ID label** on the PCB (either as a sticker or printed on the silkscreen).

Take a moment to:
- Locate the ID on each motor
- Find the corresponding ID on the PCB

⚠️ **Important:** Correct ID matching is critical. Installing the wrong motor in a position will cause control issues later.

<p align="center">
    <img src="assem_motor1.jpg" width="75%">
</p>

---

### 10. Motor configuration

If this is your **first time using brand new motors**, please pause here and complete the [**Initial Motor Configuration**](#initial-motor-configuration) steps located at the end of this document **before continuing**. After that, jumps to step #13 directly.

If you are working with a **pre-configured workshop kit** (or you have been told that the motors are already configured), you may proceed with the steps below.

---

### 11. Install the first motor

Let's start with **motor #11**.

1. With the **bottom shell removed**, carefully align the **PCB headers** with the motor’s pin header.
2. Press the PCB straight onto the motor until it is fully seated.
3. Once aligned, use the long screws (saved earlier) to secure the motor in place.  
   - Only **2 screws are required**, installed **diagonally**. Refer to the hole locations on the 3D-printed frame.

<p align="center">
  <img src="assem_motor2.jpg" width="45%">
  <img src="assem_motor3.jpg" width="45%">
</p>

*(Note: Please disregard the USB cable shown in the images. You will not need this connection during this step.)*

---

### 12. Repeat for remaining motors

Repeat **Step 9** for the remaining motors:

**Motor IDs:** #11 through #18

Work slowly and double-check:
- ID matching
- Header alignment
- Screws are snug, **not overtightened**

---

### 13. Leg installation

Once all motors are connected, switch the power back on by sliding the switch towards the battery. Within a few seconds, all motors should slowly move to an initial position that makes installation easy.

Install the legs. Make sure one identical pair of legs is installed on the left, and the other mirrored pair on the right. Refer to the CAD model of the robot if you find this confusing.

<p align="center">
    <img src="assem_attachlegs.jpg" width="75%">
</p>

🎉 **Congratulations!** You have finished assembling your **Q8bot** 🤖  

You may now **turn off the robot power**, or directly proceed to [Software Setup](software_setup.md) 💻

---

<br><br>

# Appendix

## Initial Motor Configuration

**Note:** The following motor setup instructions only apply to **brand new DYNAMIXEL motors** that have not been configured before.

#### 10.1 Software setup

The next steps involve the use of [PlatformIO](https://platformio.org/) on Microsoft Visual Studio Code to configure the smart servo motors. If you haven't used it before, please refer to their official documentation and tutorials to set up the environment.

Download or clone the `q8bot` repository and open the following folder:
`firmware/q8bot_motor_config`.

With your battery installed and power turned on, connect your Q8bot PCB to your laptop using a USB-C cable.

#### 10.2 Select the COM port

Select the correct COM port by clicking the button circled in red, or leave it set to **“Auto”**, which will detect the correct port most of the time.

<p align="center">
    <img src="assem_port.jpg" width="75%">
</p>

#### 10.3 Upload the motor configuration firmware

Upload the `q8bot_motor_config` firmware to the Q8bot PCB by clicking the button with an arrow shape (circled in red).

<p align="center">
    <img src="assem_upload.jpg" width="75%">
</p>

#### 10.4 Open the serial monitor

If the upload is successful, open the serial monitor by clicking the button with the shape of a cable plug.

<p align="center">
    <img src="assem_serial.jpg" width="75%">
</p>

#### 10.5 Verify no motors are detected

In the serial monitor, you should see the message  
**“No new Dynamixel motors found”**  
printed approximately every 5 seconds.

<p align="center">
    <img src="assem_nofound.jpg" width="75%">
</p>

#### 10.6 Attach and configure the first motor

We will now attach and configure the motors one at a time.

Grab one new motor with its **back shell removed**, and locate the position on the PCB marked **“ID 11”**.  
(The latest published Gerber files have the IDs printed directly on the silkscreen, rather than using a separate label as shown in the image.)

<p align="center">
    <img src="assem_motor1.jpg" width="75%">
</p>

#### 10.7 Detect and assign motor ID

Align the PCB headers with the motor pins and install the first motor **without** screwing it in place.

As soon as the motor is connected, you should see the following message appear in the serial monitor:

    ```New motor connected! DXL Params set. Changing ID to 11.```
<p align="center">
    <img src="assem_motor2.jpg" width="75%">
</p>

#### 10.8 Secure the motor

You may now use the long screws from detaching the shells to secure the first motor in place.  
Note that **only 2 screws** are needed instead of 4.

**Note:** In some PCBWay assembly services, longer (black) female pin headers may be used and can be too tall. In this case, you may use a flush cutter or sandpaper to trim the headers down to an appropriate height.

<p align="center">
    <img src="assem_motor3.jpg" width="75%">
</p>

#### 10.9 Configure remaining motors

Repeat **Steps 10.6–10.8** for the remaining motors (**ID 12 through ID 18**).

Be sure to follow the labeling numbers **sequentially**, as the calibration code is written to only look for the next available motor.

<p align="center">
    <img src="assem_motor4.jpg" width="75%">
</p>

#### 10.10 Final verification

Once all motors are connected, the program should automatically move all motors in preparation for leg installation.

You may disconnect the robot from the laptop now, but **keep it powered on**.

<p align="center">
    <img src="assem_motorcomplete.jpg" width="75%">
</p>