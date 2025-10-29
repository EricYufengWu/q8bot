# Q8bot Executable Build Instructions

This guide explains how to build a single-file executable for Q8bot.

## Prerequisites

1. **Install Python dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

2. **Install PyInstaller:**
   ```bash
   pip install pyinstaller
   ```

## Building the Executable

### Windows

Run the build script:
```cmd
.\build_exe.bat
```

Or manually:
```cmd
python -m PyInstaller q8bot_build.spec
```

**Output:** `dist/q8bot.exe`

### Linux / macOS

Make the script executable and run it:
```bash
chmod +x build_exe.sh
./build_exe.sh
```

Or manually:
```bash
python -m PyInstaller q8bot_build.spec
```

**Output:** `dist/q8bot`

## Running the Executable

### Windows
```cmd
# Auto-detect COM port
dist\q8bot.exe

# Specify COM port
dist\q8bot.exe COM3

# Enable debug logging
dist\q8bot.exe --debug

# Specify COM port with debug logging
dist\q8bot.exe COM3 --debug
```

### Linux / macOS
```bash
# Auto-detect serial port
./dist/q8bot

# Specify serial port
./dist/q8bot /dev/ttyUSB0

# Enable debug logging
./dist/q8bot --debug
```

## Build Configuration

The build is configured in `q8bot_build.spec`:

- **Single file:** All dependencies bundled into one executable
- **Console mode:** Shows console for logging output
- **Embedded config:** Joystick configuration is embedded in Python code (no external files needed)
- **Dependencies:** numpy, scipy, pygame, pyserial
- **Excluded:** matplotlib, PIL, sympy (not needed)
- **UPX compression:** Enabled for smaller file size

## Troubleshooting

### "Module not found" errors
- Ensure all dependencies are installed: `pip install -r requirements.txt`
- Check that `hiddenimports` in spec file includes missing modules

### Missing joystick configuration
- The joystick configuration is now embedded directly in `control_config.py`
- No external JSON file is needed or bundled

### Executable won't run
- Make sure Visual C++ Redistributables are installed (Windows)
- On Linux, ensure executable permissions: `chmod +x dist/q8bot`

### Large file size
- UPX compression is enabled but can be improved
- Consider excluding unused scipy/numpy submodules if needed

## Distribution

The generated executable is self-contained and can be distributed to other computers with the same OS. Users do NOT need Python installed.

**Note:** You still need to distribute for each platform separately:
- Windows users need `q8bot.exe` (built on Windows)
- Linux users need `q8bot` (built on Linux)
- macOS users need `q8bot` (built on macOS)
