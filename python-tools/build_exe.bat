@echo off
REM Build Q8bot single-file executable using PyInstaller
REM
REM Prerequisites:
REM   pip install pyinstaller
REM
REM Output:
REM   dist/q8bot.exe

echo ========================================
echo Building Q8bot Executable
echo ========================================
echo.

REM Check if PyInstaller is installed
python -c "import PyInstaller" 2>nul
if errorlevel 1 (
    echo ERROR: PyInstaller not found!
    echo Please install: pip install pyinstaller
    echo.
    pause
    exit /b 1
)

REM Clean previous builds
echo Cleaning previous builds...
if exist build rmdir /s /q build
if exist dist rmdir /s /q dist

REM Build executable
echo.
echo Building executable...
python -m PyInstaller q8bot_build.spec

REM Check if build succeeded
if exist dist\q8bot.exe (
    echo.
    echo ========================================
    echo Build successful!
    echo ========================================
    echo.
    echo Executable location: dist\q8bot.exe
    echo.
    echo Usage:
    echo   dist\q8bot.exe           - Auto-detect COM port
    echo   dist\q8bot.exe COM3      - Use specific COM port
    echo   dist\q8bot.exe --debug   - Enable debug logging
    echo.
) else (
    echo.
    echo ========================================
    echo Build FAILED!
    echo ========================================
    echo Please check the output above for errors.
    echo.
)

pause
