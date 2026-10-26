# ID18 diagnostic results — 2026-09-09

The observations strongly suggest a local hardware problem in ID18. Its
communication and position sensing work, but it did not turn under the same
small position test that successfully moved ID16. The exact failed component
has not been established; an internal motor/driver problem is a possibility,
and a physical inspection is still needed.

## Conditions and passive checks

The user reported the robot supported in the air, with battery power on,
robot USB connected, and Pygame closed. The robot ran temporary diagnostic
firmware independently of the walking/rearing Python code.

- All eight servos responded consistently, with no hardware error flags.
- ID18 reported 4.9 V and 26–28 °C across the observations.
- ID18's position changed from 5124 to 6073 ticks after the user moved the
  linkage by hand. This confirms its position reading responds to movement;
  it does not establish motor or gear health.
- ID16 and ID18 had P/I/D gains 400/0/400, PWM Limit and Goal PWM 885, and
  Current Limit 1750 mA. Both were in extended position mode 4. Drive modes
  and homing offsets matched their respective mirrored Q8 configurations.
- A prior power cycle may have cleared the original fault flags.

## Active comparison

Each test activated only its compiled target servo. All eight had to respond
with torque off before starting. Goal PWM was reduced to at most 133 (about
15% duty), without increasing any existing limit. The target moved down by
34 ticks (about 3°) over one second, followed by a return over one second;
each stage was observed for 1.4 seconds. No mode, calibration, EEPROM limit,
or shutdown-mask changes were made.

| Observation | ID18 | ID16 comparison |
| --- | --- | --- |
| Start → requested target (ticks) | 5789 → 5755 | 5689 → 5655 |
| Measured outward position range | 5789 only | 5689 down to 5656 |
| Last return position | 5789 | 5685 (4 ticks from start) |
| Measured current range | −5 to 0 mA | −13 to +13 mA |
| Largest absolute reported PWM | 54 | 27 |
| Servo supply during test | 4.9 V | 4.9–5.0 V |
| Torque readback during test | 1 | 1 |
| End state | Torque off; settings restored | Torque off; settings restored |

ID18's position stayed unchanged in all 60 active samples, despite accepting
the goal and reporting nonzero PWM. ID16 moved about 2.9° under the same
bounded test. This makes the Python gait logic an unlikely explanation for
ID18's present failure. The settings and unloaded supply observations do not
show an obvious configuration or idle-voltage cause.

These tests do not prove that ID18 caused the earlier complete power losses,
or establish whether the added weight/rearing poses contributed to damage.
USB was connected, and neither walking loads nor battery voltage transients
were tested. Successful unloaded movement also does not validate full strength.

## Firmware and next step

The normal `robot_permanent` firmware was rebuilt and uploaded successfully
to robot MAC 3C:DC:75:AE:E9:FC, with flash hashes verified. The restore used
the original servo-control source from commit
`f86fefca7d849c4a19278cbb44090dd24187b81c` in a temporary build directory.
The previously prepared, unuploaded profile-verification edits remain in the
working tree and were not included in this restoration. Pygame was not started.
USB serial remained available after restoration, but no further console output
was captured during the eight-second observation. Normal wireless operation was
not exercised with the faulty servo.

All three diagnostic firmware environments built successfully. Host checks for
both target IDs passed, covering normal sequencing, failed preflight/read/write
checks, overcurrent/hardware-alert aborts, requested stop, output restoration
only after torque-off verification, and rejection of unwanted serial commands.

Power off and unplug USB before inspecting ID18's connector, servo horn and
linkage. If those connections are sound, arrange servo service or replacement
before further walking/rearing tests.

Raw evidence: [powered scan](powered-scan.log),
[manual movement](manual-movement-scan.log),
[ID18 active test](id18-movement.log),
[ID16 comparison](id16-comparison.log),
[restored firmware serial observation](restored-firmware.log).

Register interpretation follows the
[ROBOTIS XL330-M077 manual](https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/),
particularly Torque Enable, Goal PWM, Present Position, Present Current, and
Hardware Error Status. The hardware diagnosis above is an inference from the
measurements, not a manufacturer-confirmed component diagnosis.
