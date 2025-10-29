#!/bin/bash
# Build Q8bot single-file executable using PyInstaller
#
# Prerequisites:
#   pip install pyinstaller
#
# Output:
#   dist/q8bot

echo "========================================"
echo "Building Q8bot Executable"
echo "========================================"
echo

# Check if PyInstaller is installed
if ! python -c "import PyInstaller" 2>/dev/null; then
    echo "ERROR: PyInstaller not found!"
    echo "Please install: pip install pyinstaller"
    echo
    exit 1
fi

# Clean previous builds
echo "Cleaning previous builds..."
rm -rf build dist

# Build executable
echo
echo "Building executable..."
python -m PyInstaller q8bot_build.spec

# Check if build succeeded
if [ -f "dist/q8bot" ]; then
    echo
    echo "========================================"
    echo "Build successful!"
    echo "========================================"
    echo
    echo "Executable location: dist/q8bot"
    echo
    echo "Usage:"
    echo "  ./dist/q8bot           - Auto-detect COM port"
    echo "  ./dist/q8bot /dev/ttyUSB0  - Use specific serial port"
    echo "  ./dist/q8bot --debug   - Enable debug logging"
    echo
else
    echo
    echo "========================================"
    echo "Build FAILED!"
    echo "========================================"
    echo "Please check the output above for errors."
    echo
    exit 1
fi
