# SQuRo hind-leg assembly session — 2026-10-12

Robot: ESP32-C3, MAC `3c:dc:75:ae:e9:fc` (read with `esptool read_mac` before
any write), USB `/dev/cu.usbmodem2101`. Head (ID19) and both front legs were
already assembled; this session fitted the two hind legs.

## Firmware

The Mac now runs macOS 27.0.1 (arm64) without Rosetta. PlatformIO's
`toolchain-riscv32-esp` 8.4.0+2021r2-patch5 is x86_64-only, so **nothing can be
compiled on this machine** until Rosetta is installed or the toolchain is
replaced. The existing `assemble_squro` build was used instead:
`firmware.bin` dated 2026-10-01 17:42:33, five seconds after the last edit of
`assemble_squro.cpp` / `squro_assembly.h`. `strings` confirmed it contains the
current banner and both `ALIGNBL` and `ALIGNBR`. It was uploaded with
`pio run -e assemble_squro -t nobuild -t upload`; flash hash verified. The
previous session's log shows the robot had been running the older front-only
build. Host checks `test_squro_assembly` and `test_head` passed.

## Preflight, 15:43

All nine servos: model 1190, correct IDs, baud register 3, operating mode 4,
configured drive modes and homing offsets, torque off, hardware errors 0,
4.9–5.0 V, 19–20 °C. Head 4081 ticks with its goal also at 4081 after the power
cycle (goal initialised to present at boot).

The user confirmed the rear mapping: ID15 (left) and ID17 (right) are the
servos of each hind pair nearer the nose — the femur pivot, carrying the elbow
crank. ID16 and ID18 are nearer the tail and carry the shoulder crank. This is
the same convention as the front, where ID11 nearest the nose was confirmed on
2026-10-01.

| ID | Start | Target | Move |
|---|---:|---:|---:|
| 15 | 4084 | 3627 | −40° |
| 16 | 6877 | 4745 | −187°, **refused by the 2048-tick rear guard** |
| 17 | 4141 | 3627 | −45° |
| 18 | 5884 | 4745 | −100° |

## Hind-right, `ALIGNBR` at 15:48:02

Ready about five seconds later: ID17 3626, ID18 4748. Peak current 15 / 18 mA,
PWM 36 / 52 of the 266 cap. ID18 held at 4745–4748 for about 3½ minutes; in
the final ~7 s it was pushed to 4778 (33 ticks against a 32-tick limit) and the
hold released, about 227 s after ready. The abort sample read current −20 mA,
PWM 79, torque on, no fault: the disturbance guard, not a servo fault. The user
reported the leg done. Torque-off on all servos was confirmed by `RELEASE` and
`LEGS`. The user was asked, but did not explicitly confirm, that both cranks
were screwed down before the release; the steady trace until the last seconds
is consistent with that.

## Hind-left, `ALIGNBL` at 16:01:53

The user turned the bare ID16 horn by hand, clockwise seen from the left:
6878 → 6286 ticks, bringing it inside the guard. Ready about five seconds later
at ID15 3628, ID16 4746. The hold released after **~46 s** when ID15 was
pushed to 3583 (44 ticks; current 26 mA, PWM 68, torque on, no fault), so most
of the hind-left fitting happened with torque off. The user reported the leg
done at about 16:05.

Keying check, read-only at 16:06 with both hind legs assembled and unpowered:
hind-left elbow −29.2° / shoulder +23.6°, hind-right −42.0° / +32.7°. The two
agree within 13°, whereas a crank fitted through the wrong screw-hole position
would sit at least 90° off. The front legs read differently from earlier in the
session because they had been moved while unpowered, so they are not used for
comparison.

The console exited at 16:06:27 with a final release, confirmed. The robot still
runs `assemble_squro`, which never moves anything on its own and releases every
responding servo at startup.

## Open items

1. **Do not operate the assembled robot from the GUI yet.** The GUI starts on
   the Q8 design and immediately drives all four legs to the Q8 rest pose; even
   with Biomimetic selected it sends the front-leg pose to the hind pair. Either
   would swing the hind legs from knee-forward to knee-back. The host needs a
   separate front and hind pose (and a hind solver for the different foot)
   first.
2. **Hind shoulder wrap point.** ID16/ID18 read 0–360° (homing offset +4096),
   so their reading wraps where the shoulder crank points horizontally toward
   the nose — only 57° from the 4745 mounting reference, against roughly 128°
   for the front servos. Both hind shoulders rested at 24–33° today. If a hind
   femur sits above horizontal-forward at power-up it reads near 360°, and the
   first command would turn it about 300° the wrong way. A check that refuses
   torque-on while any joint is far from its first commanded target is needed
   before powered operation.
3. **Toolchain.** Install Rosetta (`softwareupdate --install-rosetta`) or move
   to a native arm64 RISC-V toolchain before rebuilding any firmware, including
   the normal robot firmware.
4. **Disturbance guard.** The 32-tick hold limit tripped during fitting on all
   four legs. A wider limit during the fitting phase would make the hold last
   through assembly; it needs a rebuild.

Serial evidence: [assembly.log](assembly.log).
