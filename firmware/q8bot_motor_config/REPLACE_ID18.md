# Replacing only ID18

The `replace_id18` PlatformIO environment configures one replacement XL330-M077-T
(model 1190), while leaving IDs 11–17 untouched. It replaces the robot application
temporarily and contains no torque-on or position commands. The original
`seeed_xiao_esp32c3` environment remains the complete eight-servo assembly program;
that program automatically moves all motors at its end.

1. Close Pygame. With battery power and USB disconnected, install the replacement
   servo in ID18's location. Leave its leg linkage detached for alignment.
2. Connect the robot's USB with the battery switch off. Upload this environment:
   `pio run -e replace_id18 -t upload --upload-port <robot USB port>`.
3. Support the robot and turn battery power on. Open serial at 115200 baud.
4. Inspect `STATUS`, then send the exact line `CONFIG18`. The helper searches
   only ID1/ID18 at 57600 and 1000000 baud. It requires exactly one candidate,
   the expected model, and the other seven configured motors present with torque
   off. Keep the old faulty motor disconnected; only one new motor may be present.
5. Wait for `CONFIG18 SUCCESS`. Values are read back after each change, including
   at the new ID/baud. Repeating the command skips already-correct settings and
   can recover an interrupted ID or baud change. A reported failure is not success.
6. Set the replacement to the assembly mounting position before attaching its
   linkage. The existing assembly code uses **5622 ticks** for ID18. This helper
   deliberately does not execute that movement; it needs a separate controlled
   alignment step with the linkage detached, followed by a check of leg orientation.
7. Restore the normal `q8bot_robot` / `robot_permanent` firmware before operating.

The target values, taken from the current automatic setup and the previous
working ID18 configuration, are ID18, baud register 3 (1 Mbps), Operating Mode 4,
Drive Mode 5 (reverse and time-based profiles), and Homing Offset **−4096**.
The repository's older manual-configuration document lists positive offsets for
the right side; this replacement helper follows the actual automatic setup and
the settings read from this robot. It does not copy that conflicting manual value.

The [ROBOTIS manual](https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/) lists
factory ID1/57600 baud and requires torque off for EEPROM configuration writes.

## Mounting alignment environment

`align_id18` includes the optional `ALIGN18` and `RELEASE18` serial commands.
It disables ID18 torque on controller startup, then waits for a command. Use it
only after configuration has succeeded and with ID18's linkage detached, the
robot supported, and hands clear of the output. `ALIGN18` moves only ID18 to
5622 ticks over four seconds with Goal PWM capped at 266 (~30% duty). It checks
readbacks and monitors current, voltage, temperature, position, PWM, torque and
hardware errors. If the target settles within six ticks, `ALIGN18 READY` signals
a hold for up to five minutes to permit mounting. `RELEASE18` or `!` releases
the servo; timeout or a failed check also releases it. A loss of holding torque
requires checking the mounting reference again before attaching the linkage.

Profiles/output are restored only after torque-off is confirmed. These are
temporary RAM writes; the alignment environment does not change calibration
or EEPROM limits during movement. Restore the normal robot firmware after
mounting and releasing the servo. Initial position must be 4400–6250 ticks;
otherwise investigate the reference before issuing movement.

Host checks:
`c++ -std=c++17 tests/test_replacement.cpp -o /tmp/q8-replacement-test && /tmp/q8-replacement-test`
and `c++ -std=c++17 tests/test_alignment.cpp -o /tmp/q8-alignment-test && /tmp/q8-alignment-test`.
