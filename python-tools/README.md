# Q8bot python-tools

Python control software for the Q8bot quadruped robot.

## Quick Start

See [Software Setup](../building_instructions/software_setup.md) for details on running from source.

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
