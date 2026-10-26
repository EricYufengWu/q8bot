# Q8bot servo diagnostics

Temporary read-only firmware for the robot's XIAO ESP32C3, to compare ID18 with
IDs 11-17. It uses the configured bus (Protocol 2.0, 1 Mbps) and does not change
servo IDs, calibration, profiles, torque, or target positions. It replaces the
robot's application firmware; restore `q8bot_robot` afterward to operate normally.

Close the control app and switch battery power off before connecting the robot
by USB and uploading. Once uploaded, power the servos from the robot's batteries
and read the serial monitor at 115200 baud. Do not flash `q8bot_motor_config`.

The scan reads hardware errors, supply voltage, temperature, current, torque,
positions and configuration. Read failures are distinct from a valid zero.
Diagnostic operation does not test motor strength, loaded voltage drop, gear
condition or movement. A power cycle can clear the fault that occurred earlier;
a zero hardware-error register does not prove the servo is healthy. If torque
was already enabled before running this firmware, read-only diagnostics will
not disable it; start with battery power off as described above.

## Optional bounded ID18 test

The default `seeed_xiao_esp32c3` build remains read-only. The separate `id18_test`
environment includes an explicitly commanded test. It has not been included in
the default build. Build with `pio run -e id18_test`; uploading this environment
replaces the read-only firmware. It disables ID18 torque on controller startup,
then only reads until the exact serial line `TEST18` is received. Never send that
command unless the body is supported, the leg can move freely, and hands are clear.

The test requires all eight servos to respond with torque off and ID18 to have
the expected stored configuration, no active fault/watchdog, and an initial
position between 4400 and 6250 ticks. It reduces Goal PWM to at most 133 (~15%
duty; this is not a current limit), commands a 34-tick (~3 degree) decrease over
one second and a return over one second, then turns torque off. Each leg is
observed for 1.4 seconds. It monitors current, PWM, position, voltage, temperature,
torque and hardware errors, aborting on a failed check/read or serial `!`.
Thresholds are conservative diagnostic choices, not manufacturer operating limits.

Only ID18 RAM registers are written. Existing shutdown protections remain active;
mode, calibration and persistent limits are not changed. Output/profile settings
are restored after torque-off is confirmed; a restoration failure is reported.
If torque-off cannot be confirmed, switch battery power off. A failed movement
at reduced PWM alone does not prove the motor is defective: friction, loading,
and insufficient test output can also prevent motion. Successful unloaded tracking
does not validate strength or the battery supply under walking/rearing load.

Host checks exercise the actual sequence with a simulated bus:
`c++ -std=c++17 tests/test_id18.cpp -o /tmp/q8-id18-test && /tmp/q8-id18-test`.

For a comparison with the corresponding left rear servo, `id16_test` compiles
the same bounded sequence exclusively for ID16, with its expected drive mode 4
and homing offset +4096. Its command is `TEST16`; `TEST18` is ignored by that
build. Each build can activate only its one compiled target ID. Run its host
checks with `c++ -std=c++17 -DQ8_TEST_ID=16 tests/test_id18.cpp -o /tmp/q8-id16-test && /tmp/q8-id16-test`.
