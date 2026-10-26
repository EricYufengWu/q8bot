# Head configuration and SQuRo assembly session — 2026-10-01

Robot: ESP32-C3, MAC `3C:DC:75:AE:E9:FC`, USB `/dev/cu.usbmodem2101`.
The user confirmed the SQuRo leg design, all leg linkages detached, and
battery power off before upload, then on for configuration.

## Completed

- Built and uploaded `config_head`; flash hashes verified.
- Head configuration host checks passed. All 30 `test_leg_designs` checks passed.
- Found exactly one new XL330-M077-T (model 1190), ID1 at 57600 baud.
- `CONFIGHEAD SUCCESS` at 17:15 local time; confirmed by a separate `STATUS`.
- Head is now ID19 at 1 Mbps, operating mode 4, drive mode 4,
  homing offset +2048, torque off, hardware errors 0.
- Head present position and parked goal both read 3983 ticks: −9.93° relative
  to the configured zero at 4096 ticks. Physical head orientation is not yet
  established. Supply 5.0 V, temperature 18 °C.
- IDs 11–18 were confirmed unchanged by the configuration helper. All had
  torque off and hardware errors 0. Supply 4.9–5.0 V, temperatures 18–20 °C.
- Configuration itself issued no torque-on. A later controlled head-centering
  movement and the first front-left alignment are recorded below.

## Leg inventory after head configuration

All eight are model 1190, baud register 3 and operating mode 4.
These positions are observations with the linkages detached, not mounting targets.

| ID | Drive mode | Homing offset | Present position |
|---|---:|---:|---:|
| 11 | 4 | +2048 | 5336 |
| 12 | 4 | +4096 | 5812 |
| 13 | 5 | −2048 | 3770 |
| 14 | 5 | −4096 | 5236 |
| 15 | 4 | +2048 | 3991 |
| 16 | 4 | +4096 | 6688 |
| 17 | 5 | −2048 | 4201 |
| 18 | 5 | −4096 | 5844 |

## Assembly checkpoint

The user reports the head is turned very slightly right, ID11 is the front-left
servo nearest the nose, and the head cable is probably chained through ID13.
The head's ID19 was independently established by serial configuration/readback.

The project's front-leg SQuRo model uses q1 = 52.660° (4695 ticks) for the
elbow crank at B and q2 = 129.021° (5564 ticks) for the shoulder crank at A.
With the user's physical check and a fresh read-only inspection of the open
Fusion CAD, the front-left mapping is B/elbow = ID11 (4695 ticks) and
A/shoulder = ID12 (5564 ticks). CAD front B is at world x −50 mm, A at −30.5 mm,
and the head is at −78 mm. The front-right mapping still needs a physical check.
The rear assembly swaps the crank locations relative to the front and has a
different foot; the current host model repeats the front-leg model for all four
legs. Rear mounting references and operating motion still need resolution.

The `assemble_squro` firmware and its host checks have now passed, including
head/front-only RAM write scope, goal parking before torque, target tracking,
fault stops, heartbeat/hold timeouts, failed release/restoration interlocks,
and command parsing. After the user again confirmed battery power off, it was
uploaded successfully with verified flash hashes. Its `CENTERHEAD` command
moves only ID19 to 4096; `ALIGNFL` moves only IDs 11/12 to the front reference.
Rear movement is unavailable. See [ASSEMBLE_SQURO.md](../../ASSEMBLE_SQURO.md).

The robot currently runs `assemble_squro`, not the normal wireless application.
The assembly console is connected and sends only heartbeats automatically.
After the user confirmed power on and hands clear, `CENTERHEAD` was issued at
17:29:53 local time. ID19 travelled from 3983 toward 4096 ticks over five seconds
and reported `ASSEMBLY READY`, settling at 4091–4092 ticks (0.35–0.44° short of
zero). During the observed move, current was 0–17 mA, PWM 0–48, supply 5.0 V,
and temperature 20 °C. The user confirmed that it was centred. `RELEASE` at
17:30:58 confirmed torque off and restored RAM profiles/output. A separate
`STATUS` read showed head position and goal both at 4092, torque 0, errors 0.

`ALIGNFL` at 17:32:05 moved ID11 from 5336 toward 4695 and ID12 from 5812
toward 5564. Both settled within the six-tick tolerance and reported
`ASSEMBLY READY` at approximately 17:32:10. Observed hold positions: ID11 =
4700, ID12 = 5566. Current magnitude stayed at or below 17 mA in the captured
move, supply was 4.9–5.0 V, temperatures 24–26 °C. Front-left fitting is now
the physical checkpoint; other leg servos remain torque-off. The hold expires
at approximately 17:37:10 unless released/aborted sooner. The user was told to
fit only the front-left linkage, with pushrods last and no forced alignment.

The user then reported "front-left fitted". On collecting the intervening
telemetry, the log showed a hold abort on ID11 after its position progressively
decreased from 4700 to 4676 ticks; the next sample triggered the generic
telemetry/read guard. The last logged current was 18 mA and PWM 29. Position
disturbance is the likely cause, but the generic abort line does not contain the
failing sample, so the exact guard cannot be proven from that line alone.

An explicit `RELEASE` and separate `LEGS` / `STATUS` at 17:37:31 confirmed all
nine servos torque-off and hardware errors 0. Front-left then read ID11 =
4495 and ID12 = 5514; head = 4090. The mechanical attachment has been reported
complete. The user subsequently confirmed both cranks were fully seated and
screwed down before holding torque released. Front-left attachment is recorded
as complete on that basis; loaded movement has not been tested. No further
movement or front-right alignment has been issued. Front-right mapping and
readiness, plus rear servo mapping, have been requested.

A further read-only CAD inspection measured front crank angles 52.660° /
129.020° (the latter differs from the solver's 129.021° humerus angle by a small
CAD residual, but rounds to the same tick). Rear crank angles are elbow
−41.286° and shoulder +57.000°, measured from the nose direction downward.
The existing robot angle conversion maps these to 3627 / 4745 ticks. These are
geometry-derived rear references, not yet assigned to verified physical IDs or
commanded. Measurements: [cad-crank-references.json](cad-crank-references.json).

Front-left mounting guide: [front-left-mount.png](front-left-mount.png).
Assembly serial evidence:
[assembly.log](assembly.log).
The head goal is a RAM value. Recheck and park it after any manual head movement
or servo power cycle before enabling torque. Keep linkages detached until their
mounting references have been verified and established.

Raw serial evidence: [head-bench.log](head-bench.log).
