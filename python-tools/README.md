# Q8bot python-tools

Python control software for the Q8bot quadruped robot.

## Quick Start

See [Software Setup](../building_instructions/software_setup.md) for details on running from source.

With the Pygame window focused, press **P** to toggle a static sitting pose:
the front legs extend and the rear legs crouch over one second. Press **P**
again to return to the selected gait's resting position. Holding P does not
repeat the toggle. Walking and other motion routines are paused while sitting;
**B** (battery) and **Esc** (exit and release torque) still work.
The pose heights and transition duration are in `q8bot/sitting_pose.py`.
The sitting targets are 60 mm at the front and 20 mm at the rear. These are
foot distances from the leg mounts in the kinematic model.

Press **U** to enter and hold a static rearing position. It uses the greeting's
entry waypoints but holds both forelegs still in the final stance, without
waving. Press **U** again to lower back to the selected gait. Entry takes about
3.7 seconds; lowering takes about 6.4 seconds. Lowering first extends the front
legs to a 60 mm support target over 2 seconds, keeping the rear targets fixed.
After a 0.2 second pause, the rear legs move to the selected gait over 2.5 seconds
while the fronts stay extended. After another 0.2 second pause, all legs settle
into that gait's resting position over 1.5 seconds. These are timed transitions,
without ground-contact sensing. The GUI displays each phase and remains responsive during
both transitions. **B** and **Esc** work throughout. Return from a static pose
before entering the other one (**P** for sitting, **U** for rearing).

The rearward adjustment subtracts 13 degrees from both rear joint angles
of the greeting support stance, giving rear pairs `[37, 62]` and fixed front
pairs `[-45, 45]`. `REARWARD_ANGLE_OFFSET` in `q8bot/rearing_pose.py` is the tuning
parameter. Balance depends on payload mass/location and floor contact: the
geometry tests do not establish that a weighted robot can hold this pose.
Support the chassis during the first physical trial. No feedback balancing is
implemented; the preview's body angle is illustrative.

On startup the GUI detects which legs are fitted before anything moves: lift
the robot so the legs hang free and press Enter. Each joint is swept gently to
its stop and back, and the stops are matched against recordings; the first time,
pick the fitted design and press R to record it. `--legs Q8` or
`--legs BIOMIMETIC` skips detection. Biomimetic legs start released: set the
robot on its feet and press **T** to stand up, **T** again to crouch. See
[Leg designs](docs/leg_designs.md) for how the sweeps are kept safe.

Press **L** to change the leg design. The picker lists Q8, Full range and
Biomimetic; Up/Down choose, Enter starts the attachment calibration for that
design, Esc cancels without changing anything. Calibration drives every joint
to the design's mounting pose and holds it there while the linkages are fitted.
Support the robot first: the feet leave the ground and torque is held until you
press Enter or Esc. See [Leg designs and attachment calibration](docs/leg_designs.md)
for the geometry, the mounting poses and the head-servo wiring.

The GUI includes a picture for all eight gaits, sitting and rearing, **for each
leg design**. The picture follows **G**, **P** and **U**, and returns to the
selected gait after lowering; fitting a different design through **L** swaps the
whole set, along with that design's gait table and static poses. These are
illustrations of the commanded geometry, not live measurements. Regenerate
every design's pictures after changing gait or pose parameters:

```bash
venv/bin/python q8bot/pose_preview.py
```

They are written to `docs/poses/<design>/`.

Run the hardware-free geometry, input, image and GUI tests:

```bash
venv/bin/python -m unittest discover -s . -p 'test_*.py' -v
```

## Building a Single Executable from Source

Follow these steps to build a standalone executable that can run without Python installed:

### 1. Clone the Repository

```bash
git clone https://github.com/yourusername/q8bot.git
cd q8bot/python-tools
```

### 2. Create a Virtual Environment (Recommended)

**Windows (Command Prompt):**
```cmd
python -m venv venv
venv\Scripts\activate
```

**Windows (PowerShell):**
```powershell
python -m venv venv
.\venv\Scripts\Activate.ps1
```

**macOS/Linux:**
```bash
python3 -m venv venv
source venv/bin/activate
```

### 3. Install Dependencies

First, install the runtime dependencies:
```bash
pip install -r requirements.txt --force-reinstall
```

Then, install PyInstaller for building the executable:
```bash
pip install pyinstaller
```

The `--force-reinstall` flag ensures all packages are properly installed even if they already exist.

### 4. Build the Executable

Run the appropriate build script for your operating system:

**Windows (Command Prompt):**
```cmd
build_exe.bat
```

**Windows (PowerShell):**
```powershell
.\build_exe.ps1
```

**macOS/Linux:**
```bash
./build_exe.sh
```

The build process will:
- Check that PyInstaller is installed
- Clean previous builds
- Package the application with all dependencies
- Embed instruction images into the executable
- Create a single-file executable

### 5. Find the Executable

The built executable will be located in:

```
dist/
├── q8bot.exe          (Windows)
└── q8bot              (macOS/Linux)
```

### Running the Executable

**Auto-detect COM port:**
```bash
./dist/q8bot
```

**Specify COM port:**
```bash
./dist/q8bot COM3              # Windows
./dist/q8bot /dev/ttyUSB0      # Linux
```

**Enable debug logging:**
```bash
./dist/q8bot --debug
```

**Combine options:**
```bash
./dist/q8bot COM3 --debug
```

## Distribution

The executable in `dist/` is fully self-contained and can be:
- Copied to other computers without Python installed
- Run directly from USB drives
- Distributed to end users

**Note:** The executable is platform-specific (Windows .exe will not run on macOS/Linux and vice versa).
