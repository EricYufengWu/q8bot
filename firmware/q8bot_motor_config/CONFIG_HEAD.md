# Configuring the daisy-chained head servo (ID 19)

The `config_head` PlatformIO environment configures the head XL330-M077-T
(model 1190) as **ID 19**, while leaving IDs 11–18 untouched. It contains no
torque-on and no position commands. Use it when the eight leg motors are
already configured and only the head is new. `LEGS` provides a read-only inventory
of IDs 11–18, including torque, position, calibration and health readings.

Do **not** use the main `seeed_xiao_esp32c3` setup program for this. It rewrites
the EEPROM of all eight working leg motors, and at the end it drives every joint
to **4618/5622 ticks — the Q8 five-bar mounting pose**, which is the wrong
reference for a SQuRo build.

## Wiring

The head sits on the front centreline, about 35 mm from either front-outer leg
servo. Daisy chain it from the free connector on whichever of those two is
convenient; both are the same distance and the Dynamixel bus does not care
about the order on the chain.

## Procedure

1. Close Pygame. With battery power off and USB disconnected, daisy chain the
   head servo. Leave all leg linkages detached.
2. Connect the robot's USB with the battery switch off, then upload:
   `pio run -e config_head -t upload --upload-port <robot USB port>`
3. Turn battery power on and open serial at 115200 baud.
4. Send `STATUS`. Expect **one** candidate (a new servo answers at ID 1 and
   57600 baud) and `Leg motors answering at 1 Mbps: 8 of 8`. If it reports more
   than one candidate, another unconfigured servo is on the bus; remove it.
5. Send `CONFIGHEAD` and wait for `CONFIGHEAD SUCCESS`. Values are read back
   after every change, including at the new ID and baud rate. Repeating the
   command skips already-correct settings and can recover an interrupted ID or
   baud change. A reported failure is not success.
6. Read the reported present position. It prints how many degrees the head sits
   from centre. While torque is still off, reposition the horn if that angle is
   larger than you want to live with. After any manual adjustment, repeat
   `CONFIGHEAD` to park the goal at the new present position.
7. Restore the normal `q8bot_robot` firmware before operating.

The command refuses to run unless IDs 11–18 all answer at 1 Mbps with torque
off, exactly one head candidate is present, and the candidate reports the right
model, no hardware errors, a sane voltage and a temperature below 45 °C. After
configuring it re-reads IDs 11–18 and reports a warning if anything about them
changed.

## Target values

ID 19, baud register 3 (1 Mbps), Operating Mode 4 (extended position), Drive
Mode 4 (normal direction, time-based profiles), Homing Offset **+2048**. The
offset is the same one the other "first of pair" motors use, so the head's
mechanical centre reads as 0° — 4096 ticks — in the robot firmware's
convention.

## Why Goal Position is parked

The robot firmware writes goal positions for IDs 11–18 only, but enables torque
with a broadcast to ID 254. The head therefore receives torque without ever
receiving a goal, and would hold whatever its goal register contained. The last
step of `CONFIGHEAD` writes Goal Position = Present Position while torque is
still off, so the first torque-on holds the head still instead of moving it.
That parked goal is a RAM value, not a permanent centring calibration. After
power cycling or moving the head by hand, read and park its goal again before
enabling head torque; this helper does not guarantee a hold across those changes.

If you configure the head with the main setup program instead, it sets the
head's goal to 4096 (centre) and the head **will** move there on the next
torque-on. Support the head in that case.

The [ROBOTIS manual](https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/)
lists factory ID 1 / 57600 baud and requires torque off for EEPROM writes.
