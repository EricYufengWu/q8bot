# Leg designs and attachment calibration

The robot can be built with different leg linkages. **L** in the Pygame window
opens the leg picker; choosing a design starts the attachment calibration for
it. Up/Down move the cursor, Enter advances, Esc cancels at any point without
changing the fitted design.

| Design | Linkage | Source model | State |
|---|---|---|---|
| Q8 | Five-bar parallel | `Q8bot_rev2_5_FDM` | Available |
| Full range | MiNIQ | `Q8bot_MiNIQ_Leg` | Declared, geometry not measured |
| Biomimetic | SQuRo crank and pushrod | `Q8bot_rev2_5_SQuRo_Leg` | Available |

## What calibration does

Calibration drives every joint to the selected design's **mounting pose** and
holds it there with torque on while the linkages are fitted. That pose is what
fixes the keying between each servo horn and its crank, so every later
commanded angle means the same thing. It is the host-side equivalent of the
one-shot move at the end of `firmware/q8bot_motor_config`, so a leg swap no
longer needs the setup firmware reflashed.

| Design | Mounting pose (q1 / q2) | Ticks |
|---|---|---|
| Q8 | 45.879° / 134.121° | 4618 / 5622 |
| Biomimetic | 52.660° / 129.021° | 4695 / 5564 |

The Q8 values are the ones the motor-config firmware already uses; a test pins
them to 4618/5622 so the two cannot drift apart.

Joint order is the one on the wire: `q1` is the motor at (d, 0), `q2` the motor
at (0, 0), repeated for FL, FR, BL, BR.

Support the robot before starting. The feet leave the ground, and the sequence
holds torque until you press Enter or Esc.

## Biomimetic (SQuRo) kinematics

Measured from the pivot-hole axes in `Q8bot_rev2_5_SQuRo_Leg`. The linkage is a
**double parallelogram**:

```
|A-P1| = |B-P2| = 15.920     |P1-P2| = |A-B| = 19.500
|B-Q1| = |E-Q2| = 15.390     |Q1-Q2| = |B-E| = 34.870
```

and B, P2, E are collinear (129.019° vs 129.021°, a 0.002° modelling residual).
Both relations hold to four decimals, which means the mechanism is
kinematically a plain **2R serial arm** with both actuators left on the body:

- the humerus angle about B equals the shoulder-crank angle, and
- the ulna angle about E equals the elbow-crank angle.

So the foot follows

```
E = B + L1 * u(theta1)
F = E + L2 * u(theta2 + FOOT_OFFSET)
```

with `L1 = 34.870 mm`, `L2 = 40.607 mm`, `FOOT_OFFSET = -26.312°` and motors at
`A = (0, 0)`, `B = (19.5, 0)`. That gives a closed-form IK where the five-bar
needs a numerical solve. Reach is an annulus of 5.737–75.477 mm about B.

`squro_solver.pivots()` returns every pivot, and the calibration diagram is
drawn from it, so the picture and the commanded pose cannot drift apart. The
test suite reproduces all eight measured pivots to under 5 µm and re-checks
both parallelogram loops across the workspace.

This models the **front** leg. Front and hind differ by more than the foot
part; see *Front and hind legs are not the same leg* below.

## Head servo

The head servo is part of the biomimetic build. In the CAD it sits at
(−78.0, 7.1, 0.0), on the front centreline, about 35 mm from **either**
front-outer leg servo. Daisy chain it from the free connector on the nearer of
those two — ID 11 or ID 13, whichever side is convenient. Both are the same
distance and the Dynamixel bus does not care about the order on the chain.

It is configured as **ID 19** with drive mode 4 and the same +2048 homing
offset as the other "first of pair" motors, so its mechanical centre reads as
0° (4096 ticks) in the robot firmware's convention.

When only the head is new, use the `config_head` environment — see
[CONFIG_HEAD.md](../../firmware/q8bot_motor_config/CONFIG_HEAD.md). It touches
ID 19 only and never moves anything. The main setup program also configures the
head, but it rewrites the EEPROM of all eight working leg motors and ends by
driving every joint to the **Q8** mounting pose, which is the wrong reference
for a SQuRo build.

It is **not** a ninth commanded joint. The ESP-NOW packet still carries eight
leg angles, so existing robot and controller firmware keep working unmodified.
The robot firmware writes goal positions for IDs 11–18 only but enables torque
with a broadcast to ID 254, so the head gets torque without ever getting a
goal: it holds whatever its goal register last contained. `CONFIGHEAD` parks
that at the present position so the first torque-on moves nothing; the main
setup program instead sets it to centre, and the head will travel there.

## Per-design gaits and pictures

Fitting a design swaps four things together: the solver, the gait table, the
two static poses and the ten illustrations. Pictures live in
`docs/poses/<design>/` and are regenerated for every available design with
`venv/bin/python q8bot/pose_preview.py`. Limb shape in both the illustrations
and the calibration diagram comes from `leg_geometry`, the same description the
solvers command, so a picture cannot show a linkage the robot will not make.

A gait row is `[stacktype, x0, y0, xrange, yrange, yrange2, s1, s2]`. Stride,
lift and step counts describe locomotion, so they carry across designs
unchanged. The stance centre does not: it is where that particular linkage
naturally hangs. The biomimetic table is the Q8 table with

- `x0` set to the design's natural stance x (33.935 mm, from the CAD pose), and
- `y0` scaled by `45.113 / 43.36`, the ratio of the two natural stance heights,

then stride and lift trimmed 5% at a time only where the joints would otherwise
swing more than 40° from the mounting pose. Three gaits needed one 5% trim.

The re-centring is the whole point. Driving the biomimetic leg with the Q8
stance centre reaches **219° on q2, about 90° past its as-designed pose**.
Re-centred, the worst swing is 39.7°, which is better centred than the five-bar
is on its own gaits (74.2° at TROT_LOW). Tests assert both: no design exceeds
the 75° five-bar ceiling, and the biomimetic table holds its tighter 40° band.

`_generate_base_trajectories` previously validated inverse kinematics by
measuring the *string length* of the rounded angle and ignored the solver's own
reachability flag, so an unreachable target silently produced the previous
pose. It now honours the flag. Neither leg currently trips it, so nothing
changed today, but a new design would have hit it.

## Front and hind legs are not the same leg

Read from the CAD without activating the document: the front leg's parts all
share one occurrence transform, so the measured pose is its true assembled
pose. The hind leg's parts each carry a different transform, and:

| | Shoulder-crank axis A | Elbow/humerus axis B | Mounting pose (B / A) | Foot part |
|---|---|---|---|---|
| Front | world x −30.5 (nearer tail) | −50.0 (nearer nose) | 52.660° / 129.021° = 4695 / 5564 | `SQ3_UlnaFoot`, round pad r ≈ 19.1 mm |
| Hind | +50.0 (nearer tail) | +30.5 (nearer nose) | −41.286° / 57.000° = 3627 / 4745 | `SQ4_UlnaFootHind`, pointed toe |

Two consequences:

1. **The servo roles are the same on both pairs; the pose is not.** A and B
   only look swapped "inner/outer" because the body centre lies between the
   pairs. Counted from the nose they match: B (elbow crank, humerus/femur
   pivot) is the servo of each pair nearer the nose, so the existing ID
   convention — odd ID nearer the nose, confirmed physically for ID11 — carries
   over to IDs 15/17. What differs is the pose: the front humerus slopes back
   from B, the hind femur slopes forward (elbow back on the forelimb, knee
   forward on the hindlimb). The host now models the hind legs separately for
   calibration, stand-up and the standing picture; gaits remain single-model
   and are therefore switched off for these legs (see Known limits).
2. **The CAD stance is splayed.** The front foot sits 24.2 mm forward of its
   mount midpoint and the hind foot 20.1 mm rearward — opposite world
   directions. The gait pipeline applies one stance offset to all four legs, so
   a single `x0` cannot express that splay.

The humerus is the same length on both (34.870 mm), and the effective distance
from elbow to ground contact very nearly matches: 59.71 mm at the front (pad
centre 40.607 plus the 19.1 mm pad radius) against 59.42 mm to the hind toe.
So the two legs are close in reach and differ mainly in contact shape and pose.

Note that `y0` for the biomimetic leg is the **pad centre**, not the ground
contact; the robot stands about 19 mm higher than the number suggests.

## Hind-leg model

The hind legs now have their own solver (`squro_solver.hind()`), built from the
same CAD records: humerus 34.870 mm, toe-tip round 59.422 mm from the knee at
186.539° from the elbow-crank direction, on the knee-forward branch of the
inverse kinematics. It reproduces the measured hind pivots to 0.2 µm, and its
mounting pose gives exactly the 3627 / 4745 ticks the hind legs were assembled
at. `LegDesign.leg_solvers()` returns one solver per leg (FL, FR, BL, BR), and
calibration sends each pair its own mounting pose.

## Leg detection at startup

Starting the GUI without `--legs` runs leg detection before any torque:

1. The robot reports all nine joint angles (read-only).
2. You lift the robot so the legs hang free and press Enter.
3. Four rounds sweep the same joint of every leg at once — elbows one way, then
   the other, then the shoulders — at about 16 % output and about 18 °/s, while
   the other joint of each leg is held. A joint stops at the first contact, at
   its bound, or on any fault; afterwards every swept joint returns to its start
   and the robot releases the legs. About half a minute in all.
4. The stop angles are matched against stops recorded for each design.

Each sweep is bounded before it starts against **every** candidate design: no
SQuRo parallelogram may come within 15° of its change point, and no Q8
five-bar may bring its knee pins within 8 mm. The firmware additionally aborts
if a held joint is pushed more than 5°, which would void those bounds.

Stop angles depend on the real parts, so a design must be **recorded** once:
when the result says "not recognised", highlight the design that is fitted and
press R, then Enter. Up to five recordings are kept per design in
`~/.q8bot/leg_signatures.json`; a design is named only when at least four stops
agree and none disagree. Matching is turn-independent, so a joint whose reading
wrapped by 360° at power-up still matches. M chooses the design manually, and
`--legs Q8` / `--legs BIOMIMETIC` skips detection altogether. The last design
used is remembered in `~/.q8bot/state.json`.

If the robot does not answer — firmware without detection, or switched off —
the GUI says so and offers the manual choice. Nothing moves in the meantime.

## Standing up (biomimetic legs)

Biomimetic legs start released. Set the robot down on its feet and press **T**:

1. The joint angles are read and each is brought onto its servo's own turn, so
   no command can spin a joint a full revolution (this resolves the hind-shoulder
   wrap risk).
2. The whole path is planned before anything moves: every foot travels in a
   straight line to its CAD stance over 24 waypoints, 2.4 s. Every waypoint must
   stay on the leg's IK branch, step no joint more than 12°, keep clear of
   change points, and push no elbow or knee further into the floor than it
   started. A leg folded the other way is refused, with a message.
3. Output is capped at about 60 % and confirmed by all eight servos; only then
   does torque come on, holding the measured pose, and the waypoints follow.
4. Once standing, full output is restored so the legs can carry the body.

**T** again lowers to a crouch with the floor 34 mm below the servo axes (belly
about 8 mm clear, elbows and knees at least 9.8 mm clear), at the same capped
output. Esc releases torque and exits.

The robot cannot lie on its belly unpowered: with the feet under the hips the
front elbows reach the floor about 27.7 mm below the axes, just below the
26.1 mm belly plate, so it rests on feet and elbows. The stand-up starts from
there, wherever the legs actually are.

## Trot gaits (biomimetic legs)

Once standing, hold **W/S** to walk and **A/D** to turn on the spot; release the
keys and the legs ease back to the standing pose. **G** switches between the two
gaits while stopped. Both have the Q8 trot's structure — sine-shaped swing, flat
push back, FL with BR and FR with BL half a cycle apart, turning by running the
sides in opposite directions — built per leg on each leg's own stance and solver.

| | slow trot | fast trot |
|---|---|---|
| stride | 16 mm | 40 mm |
| lift | 8 mm | 20 mm |
| walking height (ground below the servo axes) | 64.2 mm, as standing | 58 mm, 6 mm lower |
| cycle | 1.92 s (48 frames, 40 ms) | 0.52 s (26 frames, 20 ms) |
| stance share | 32 of 48 frames | 16 of 26 frames |
| body speed | ~12 mm/s | ~125 mm/s |
| foot reach | front +6 mm, hind −6 mm | front +20 mm, hind −20 mm |
| start / stop ease | 0.4 s | 0.5 s |

The fast trot's front foot lands 40 mm ahead of the standing pose. Both
stance shares stay above half the cycle, so both diagonal pairs are briefly on
the ground together at each change.

**What limits the stride.** Front and hind legs of one side swing in the same
plane, half a cycle apart, so a long stride carries the front leg's hip pushrod
and the hind leg's forward-reaching knee pushrod toward each other. Standing,
they are 21.5 mm apart, centreline to centreline. Moving the stride window —
front feet forward, hind feet back — opens that gap.

**What limits the front reach.** As the front foot goes forward at standing
height the front leg straightens and its knee parallelogram approaches its
change point: 29° of margin at 16 mm of reach, 18° at 20 mm, 3° at 24 mm.
Walking a little lower keeps the leg bent and allows more reach; that is why
the fast trot walks 6 mm below its standing height. Below about 58 mm the gain
is small.

**Margins, for every frame of every direction of every gait**, generous because
the legs have play from loose screws:

| margin | required | slow | fast |
|---|---|---|---|
| front/hind leg gap, centrelines, side view | ≥ 18 mm | 20.4 | 19.4 |
| from a parallelogram change point | ≥ 25° | 48 | 29 |
| elbow/knee fold angle | ≥ 40° | 67 | 54 |
| elbow/knee above the floor | ≥ 8 mm | 30 | 23 |
| joint speed, peak | ≤ 800°/s | 69 | 672 |

The joint-speed limit is about a third of the XL330-M077's no-load speed. The
leg-gap check deliberately ignores that the CAD puts some of those parts in
different layers (the two pushrods are 0.5 mm apart in depth): play can tilt
them. A gait that breaks any margin refuses to load and says which. Pushing
further means giving up margin: with these limits a faster configuration
(~136 mm/s) exists only with a 38 mm stride, and a 42–44 mm stride only with
no slack on the change-point margin.

An earlier version of the slow trot, with no reach, brought the two pushrods to
14.7 mm; the 6 mm reach is what restores the margin.

## Head servo

The head servo is part of the biomimetic build. In the CAD it sits at
(−78.0, 7.1, 0.0), on the front centreline, about 35 mm from **either**
front-outer leg servo. Daisy chain it from the free connector on the nearer of
those two — ID 11 or ID 13, whichever side is convenient. Both are the same
distance and the Dynamixel bus does not care about the order on the chain.

It is configured as **ID 19** with drive mode 4 and the same +2048 homing
offset as the other "first of pair" motors, so its mechanical centre reads as
0° (4096 ticks) in the robot firmware's convention.

When only the head is new, use the `config_head` environment — see
[CONFIG_HEAD.md](../../firmware/q8bot_motor_config/CONFIG_HEAD.md). It touches
ID 19 only and never moves anything. The main setup program also configures the
head, but it rewrites the EEPROM of all eight working leg motors and ends by
driving every joint to the **Q8** mounting pose, which is the wrong reference
for a SQuRo build.

It is **not** a ninth commanded joint. The ESP-NOW packet still carries eight
leg angles, so existing robot and controller firmware keep working unmodified.
The robot firmware writes goal positions for IDs 11–18 only but enables torque
with a broadcast to ID 254, so the head gets torque without ever getting a
goal: it holds whatever its goal register last contained. `CONFIGHEAD` parks
that at the present position so the first torque-on moves nothing; the main
setup program instead sets it to centre, and the head will travel there.

## Per-design gaits and pictures

Fitting a design swaps four things together: the solver, the gait table, the
two static poses and the ten illustrations. Pictures live in
`docs/poses/<design>/` and are regenerated for every available design with
`venv/bin/python q8bot/pose_preview.py`. Limb shape in both the illustrations
and the calibration diagram comes from `leg_geometry`, the same description the
solvers command, so a picture cannot show a linkage the robot will not make.

A gait row is `[stacktype, x0, y0, xrange, yrange, yrange2, s1, s2]`. Stride,
lift and step counts describe locomotion, so they carry across designs
unchanged. The stance centre does not: it is where that particular linkage
naturally hangs. The biomimetic table is the Q8 table with

- `x0` set to the design's natural stance x (33.935 mm, from the CAD pose), and
- `y0` scaled by `45.113 / 43.36`, the ratio of the two natural stance heights,

then stride and lift trimmed 5% at a time only where the joints would otherwise
swing more than 40° from the mounting pose. Three gaits needed one 5% trim.

The re-centring is the whole point. Driving the biomimetic leg with the Q8
stance centre reaches **219° on q2, about 90° past its as-designed pose**.
Re-centred, the worst swing is 39.7°, which is better centred than the five-bar
is on its own gaits (74.2° at TROT_LOW). Tests assert both: no design exceeds
the 75° five-bar ceiling, and the biomimetic table holds its tighter 40° band.

`_generate_base_trajectories` previously validated inverse kinematics by
measuring the *string length* of the rounded angle and ignored the solver's own
reachability flag, so an unreachable target silently produced the previous
pose. It now honours the flag. Neither leg currently trips it, so nothing
changed today, but a new design would have hit it.

## Front and hind legs are not the same leg

Read from the CAD without activating the document: the front leg's parts all
share one occurrence transform, so the measured pose is its true assembled
pose. The hind leg's parts each carry a different transform, and:

| | Shoulder-crank axis A | Elbow/humerus axis B | Mounting pose (B / A) | Foot part |
|---|---|---|---|---|
| Front | world x −30.5 (nearer tail) | −50.0 (nearer nose) | 52.660° / 129.021° = 4695 / 5564 | `SQ3_UlnaFoot`, round pad r ≈ 19.1 mm |
| Hind | +50.0 (nearer tail) | +30.5 (nearer nose) | −41.286° / 57.000° = 3627 / 4745 | `SQ4_UlnaFootHind`, pointed toe |

Two consequences:

1. **The servo roles are the same on both pairs; the pose is not.** A and B
   only look swapped "inner/outer" because the body centre lies between the
   pairs. Counted from the nose they match: B (elbow crank, humerus/femur
   pivot) is the servo of each pair nearer the nose, so the existing ID
   convention — odd ID nearer the nose, confirmed physically for ID11 — carries
   over to IDs 15/17. What differs is the pose: the front humerus slopes back
   from B, the hind femur slopes forward (elbow back on the forelimb, knee
   forward on the hindlimb). The host now models the hind legs separately for
   calibration, stand-up and the standing picture; gaits remain single-model
   and are therefore switched off for these legs (see Known limits).
2. **The CAD stance is splayed.** The front foot sits 24.2 mm forward of its
   mount midpoint and the hind foot 20.1 mm rearward — opposite world
   directions. The gait pipeline applies one stance offset to all four legs, so
   a single `x0` cannot express that splay.

The humerus is the same length on both (34.870 mm), and the effective distance
from elbow to ground contact very nearly matches: 59.71 mm at the front (pad
centre 40.607 plus the 19.1 mm pad radius) against 59.42 mm to the hind toe.
So the two legs are close in reach and differ mainly in contact shape and pose.

Note that `y0` for the biomimetic leg is the **pad centre**, not the ground
contact; the robot stands about 19 mm higher than the number suggests.

## Hind-leg model

The hind legs now have their own solver (`squro_solver.hind()`), built from the
same CAD records: humerus 34.870 mm, toe-tip round 59.422 mm from the knee at
186.539° from the elbow-crank direction, on the knee-forward branch of the
inverse kinematics. It reproduces the measured hind pivots to 0.2 µm, and its
mounting pose gives exactly the 3627 / 4745 ticks the hind legs were assembled
at. `LegDesign.leg_solvers()` returns one solver per leg (FL, FR, BL, BR), and
calibration sends each pair its own mounting pose.

## Leg detection at startup

Starting the GUI without `--legs` runs leg detection before any torque:

1. The robot reports all nine joint angles (read-only).
2. You lift the robot so the legs hang free and press Enter.
3. Four rounds sweep the same joint of every leg at once — elbows one way, then
   the other, then the shoulders — at about 16 % output and about 18 °/s, while
   the other joint of each leg is held. A joint stops at the first contact, at
   its bound, or on any fault; afterwards every swept joint returns to its start
   and the robot releases the legs. About half a minute in all.
4. The stop angles are matched against stops recorded for each design.

Each sweep is bounded before it starts against **every** candidate design: no
SQuRo parallelogram may come within 15° of its change point, and no Q8
five-bar may bring its knee pins within 8 mm. The firmware additionally aborts
if a held joint is pushed more than 5°, which would void those bounds.

Stop angles depend on the real parts, so a design must be **recorded** once:
when the result says "not recognised", highlight the design that is fitted and
press R, then Enter. Up to five recordings are kept per design in
`~/.q8bot/leg_signatures.json`; a design is named only when at least four stops
agree and none disagree. Matching is turn-independent, so a joint whose reading
wrapped by 360° at power-up still matches. M chooses the design manually, and
`--legs Q8` / `--legs BIOMIMETIC` skips detection altogether. The last design
used is remembered in `~/.q8bot/state.json`.

If the robot does not answer — firmware without detection, or switched off —
the GUI says so and offers the manual choice. Nothing moves in the meantime.

## Standing up (biomimetic legs)

Biomimetic legs start released. Set the robot down on its feet and press **T**:

1. The joint angles are read and each is brought onto its servo's own turn, so
   no command can spin a joint a full revolution (this resolves the hind-shoulder
   wrap risk).
2. The whole path is planned before anything moves: every foot travels in a
   straight line to its CAD stance over 24 waypoints, 2.4 s. Every waypoint must
   stay on the leg's IK branch, step no joint more than 12°, keep clear of
   change points, and push no elbow or knee further into the floor than it
   started. A leg folded the other way is refused, with a message.
3. Output is capped at about 60 % and confirmed by all eight servos; only then
   does torque come on, holding the measured pose, and the waypoints follow.
4. Once standing, full output is restored so the legs can carry the body.

**T** again lowers to a crouch with the floor 34 mm below the servo axes (belly
about 8 mm clear, elbows and knees at least 9.8 mm clear), at the same capped
output. Esc releases torque and exits.

The robot cannot lie on its belly unpowered: with the feet under the hips the
front elbows reach the floor about 27.7 mm below the axes, just below the
26.1 mm belly plate, so it rests on feet and elbows. The stand-up starts from
there, wherever the legs actually are.

## Test trot (biomimetic legs)

Once standing, hold **W/S** to walk and **A/D** to turn on the spot; release
the keys and the legs ease back to the standing pose. The trot has the Q8
trot's structure — sine-shaped swing, flat push back, FL with BR and FR with BL
half a cycle apart, turning by running the sides in opposite directions — but is
built per leg on each leg's own stance and solver, and deliberately gentle:

| | Q8 TROT | biomimetic test trot |
|---|---|---|
| stride | 40 mm | 16 mm |
| lift | 20 mm | 8 mm |
| cycle | ~0.3 s | 1.9 s (48 frames, 40 ms apart) |
| start / stop | jumps to the first frame | eases over 0.4 s |

Every frame of every direction is checked when the gait loads: IK on the leg's
own branch, no linkage near a change point, elbows and knees at least 4 mm off
the floor, at most 6° per joint between frames. Joints stay within about ±12°
of the standing pose. A diagonal pair is always on the ground. Walking is only
accepted while standing, and T is refused while the walking keys are held.

## Head

**←/→** turn the head, **↓** centres it, **Y** swaps which way is left (the
mounting decides the sign; the choice is saved). The head never moves on its
own: nothing is sent until a key is pressed.

Its limits are measured, not assumed: after the leg sweeps, detection sweeps
the head gently each way (same low output, within ±90°) to its first contact,
and the limits sit 8° inside those contacts. They are saved in
`~/.q8bot/head.json`; until the first measurement only ±15° is allowed. The
robot clamps the head to ±90° regardless and caps its output at about 40 %, so
a misjudged limit ends in a soft bump. Commanding the head torques only ID 19,
with its goal parked first, so released legs stay released.

The limits are measured with the legs hanging; a leg pose that raises a foreleg
into the head's path is not considered.

## Robot firmware additions

`firmware/q8bot_robot` gains four commands in the existing CSV packet; the
controller firmware is unchanged, since it forwards every robot reply as one
line of 100 numbers.

| Special | Request fields | Reply (first value) |
|---|---|---|
| 6 telemetry | — | 61001: per servo ID 11–19 flags, position, current, error |
| 7 probe | 8 signed sweep bounds (deg), Goal PWM in the profile field | 61002: status, then per joint reason, start, stop, peak current, deflection |
| 8 abort | — | (the running probe ends with status "aborted") |
| 9 output cap | Goal PWM in the first field | 61003: applied value, mask of confirming servos |
| 10 head yaw | angle (deg) in the first field, move time (ms) in the profile field | none |
| 11 head probe | signed sweep bound (deg) in the first field, Goal PWM in the profile field | 61002, head in slot 8 |

Probes run in their own task, so heartbeats keep flowing; while one runs the
command parser drops every packet except battery and abort, so nothing else
reaches the servo bus. Unknown special commands no longer fall through to being
treated as joint angles. Host check of the probe against a simulated bus:
`c++ -std=c++17 -Wall -Wextra -I include test_host/test_leg_probe.cpp -o /tmp/q8-probe && /tmp/q8-probe`.

## Known limits

- **Biomimetic legs have the two trots only.** The single-model gait table,
  sitting targets and rearing waypoints stay off. Neither trot has balance
  feedback; the fast trot has not yet been run on the robot.
- **Few joints meet a stop inside the safe sweep range.** The first real run
  (2026-10-12) found contacts on 5 of 16 sweeps; the rest swung freely to their
  bound. Detection names a design from 4 agreeing contacts, so more recordings
  are needed to show those 5 recur.
- **Detection has to be recorded on the robot** before it can name a design. Q8
  can be recorded only when Q8 legs are refitted.
- **Stops depend on the held partner's angle.** The legs hang about the same way
  each time the robot is lifted, so stops should recur; if they spread, the
  tolerance widens with the spread of the recordings.
- **Stand-up and crouch are open-loop timed moves.** There is no ground-contact
  sensing or balance feedback, and the first trials should be supervised with a
  hand ready to catch the robot.
- **Full range (MiNIQ)** appears in the picker but cannot be selected until its
  pivot geometry is read out of the MiNIQ Fusion model the same way.
