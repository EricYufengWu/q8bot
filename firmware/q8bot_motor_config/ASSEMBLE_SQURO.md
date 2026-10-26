# SQuRo head and leg bench alignment

Use `assemble_squro` after head configuration. It moves one selected leg pair
or the head, with no automatic movement, EEPROM changes or broadcast torque-on.
The `config_head` environment remains configuration-only.

All four SQuRo legs were fitted with this tool: head and front pairs on
2026-10-01, hind pairs on 2026-10-12 (see `results/`). The host GUI still
repeats the front model across all four legs, so it is neither a rear assembly
tool nor safe for operating a robot with hind legs fitted.

## Front mounting reference

The user confirmed that ID11 is the front-left servo nearest the nose on
2026-10-01. A read-only inspection of the open `Q8bot_rev2_5_SQuRo_Leg` CAD
confirmed the front elbow/humerus pivot at world x −50 mm and shoulder pivot
at −30.5 mm. The head lies farther forward, at −78 mm.

| Front-left location | Part / function | Angle | Goal ticks |
|---|---|---:|---:|
| ID11, nearest nose | Elbow crank, B; humerus pivots around the same axis | 52.660° | 4695 |
| ID12, nearer body centre | Shoulder crank, A | 129.021° | 5564 |

Angles are measured from a horizontal line toward the nose, positive downward.
At this reference the shoulder crank and humerus point rearward/downward;
the elbow crank points forward/downward. The front foot is the round-pad
`SQ3_UlnaFoot`, not the pointed hind foot. The front-right pair uses the mirrored
parts and the same tick references; first verify that ID13 is nearest the nose
and ID14 nearer the body centre.

## Rear mounting reference

Same servo convention as the front: the servo of each pair nearer the nose
carries the elbow crank and is the femur pivot (B); the one nearer the tail
carries the shoulder crank (A). Confirmed physically for both hind pairs on
2026-10-12. The pose differs: the hind femur slopes forward where the front
humerus slopes back.

| Hind-left location | Part / function | Angle | Goal ticks |
|---|---|---:|---:|
| ID15, nearer nose | Elbow crank, B; femur pivots around the same axis | −41.286° | 3627 |
| ID16, nearer tail | Shoulder crank, A; parallel to the femur | +57.000° | 4745 |

At this reference the elbow crank points toward the nose and up; the shoulder
crank and femur point toward the nose and down. The hind-right pair (ID17/18)
uses the mirrored `_N` parts and the same ticks. Angles are from the CAD
(`results/2026-10-01/cad-crank-references.json`). The hind foot is the pointed
`SQ4_UlnaFootHind`.

A rear move larger than 2048 ticks (180°) is refused. A detached hind horn can
sit just beyond that: ID16 read 6877 against 4745 on 2026-10-12. Turning the
bare horn by hand into range (clockwise seen from the robot's left lowers
ID16's reading) avoids a rebuild.

## Bench sequence

1. Support the robot and head. Keep the leg linkages detached. Switch battery
   power off, leave robot USB connected, and close any other serial console.
2. Upload `pio run -e assemble_squro -t upload --upload-port <robot USB port>`.
3. Run the console with the PlatformIO Python (which has pyserial):
   `python tools/assembly_console.py --port <robot USB port> --log results/<session>/assembly.log`.
   It sends a heartbeat once per second; it never initiates movement itself.
4. Switch battery power on. Use `STATUS` and `LEGS` to inspect all nine servos.
5. With the head path clear, `CENTERHEAD` commands ID19 to 4096 ticks over five
   seconds, only if its initial position is within 256 ticks (22.5°) of zero.
   Wait for `ASSEMBLY READY` and inspect physical forward alignment. `RELEASE`
   turns torque off before proceeding to the legs. A head horn mounting error
   is not resolved merely by reaching 4096 ticks.
6. After confirming the front-left mapping and with hands clear, `ALIGNFL`
   commands ID11/ID12 to 4695/5564 over five seconds. A move exceeding 1536 ticks
   (135°) from either current position is refused.
7. Wait for **measured** `ASSEMBLY READY`, then fit the front-left cranks and
   linkage to the CAD reference. Fit the pushrods last and check that both
   parallelograms close without forcing a part. The outputs hold for at most
   five minutes. Stop on any disturbance or loss of torque; re-establish and
   verify the reference before continuing attachment.
8. `RELEASE` or `!` releases the selected pair. Verify release, then repeat with
   `ALIGNFR` only after checking the front-right mapping.
9. For the hind legs, confirm that ID15/ID17 are the servos nearer the nose,
   then use `ALIGNBL` / `ALIGNBR` the same way. `LEGS` first: each hind servo
   must be within 2048 ticks of its target or the command is refused.

The 32-tick hold guard tripped while fitting each of the four legs. Fitting
order and seating still decide the keying: a crank fitted through the wrong
screw-hole position shows up as a leg at least 90° off its mirror partner in a
read-only `LEGS` comparison.

Only `KEEP`, `RELEASE`, and `!` are accepted during movement/hold. Full scans
are blocked to preserve telemetry polling. Loss of heartbeat for 10 seconds,
failure to settle within eight seconds, or a five-minute hold timeout releases
the selected outputs. MCU startup individually releases any responding ID11–19.

## Limits and verification

All nine servos must have the expected model/calibration, torque off, watchdog 0,
no hardware errors, 3.5–5.5 V, and temperature below 45 °C before starting.
Head Goal PWM is capped at 133 (about 15% duty); front servos at 266 (about 30%),
or any lower existing Goal PWM / PWM Limit. These are output caps, not current
limits. Sampled current above ±150 mA for the head or ±250 mA for front servos
causes release, as do failed telemetry reads, unexpected position, lost torque,
temperature/voltage faults or excess PWM. These thresholds are conservative
bench choices, not manufacturer operating limits.

Targets must settle within six ticks for 200 ms after the move time. More than
32 ticks of disturbance during hold causes release. Profiles and Goal PWM are
restored only after **every selected servo** is confirmed torque-off. Failed
release/restoration blocks subsequent movement until corrected. `RELEASE` can
be retried; an unconfirmed torque-off requires battery power off.

The routine parks each selected goal at its measured position before torque-on,
then writes its mounting goal. It does not restore old goal positions. Servo
power cycles and hand movement can invalidate parked goals, so normal robot
startup/torque behavior still needs checking before operating the assembled robot.

Register interpretation: [ROBOTIS XL330-M077 manual](https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/).

Host verification:
`c++ -std=c++17 -Wall -Wextra tests/test_squro_assembly.cpp -o /tmp/q8-squro-test && /tmp/q8-squro-test`.
Firmware build: `pio run -e assemble_squro`.
