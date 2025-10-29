# Build Q8bot single-file executable using PyInstaller
# PowerShell version
#
# Prerequisites:
#   pip install pyinstaller
#
# Output:
#   dist/q8bot.exe

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "Building Q8bot Executable" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host

# Check if PyInstaller is installed
try {
    python -c "import PyInstaller" 2>$null
    if ($LASTEXITCODE -ne 0) {
        throw "PyInstaller not found"
    }
} catch {
    Write-Host "ERROR: PyInstaller not found!" -ForegroundColor Red
    Write-Host "Please install: pip install pyinstaller" -ForegroundColor Yellow
    Write-Host
    Read-Host "Press Enter to exit"
    exit 1
}

# Clean previous builds
Write-Host "Cleaning previous builds..." -ForegroundColor Yellow
if (Test-Path "build") { Remove-Item -Recurse -Force "build" }
if (Test-Path "dist") { Remove-Item -Recurse -Force "dist" }

# Build executable
Write-Host
Write-Host "Building executable..." -ForegroundColor Yellow
python -m PyInstaller q8bot_build.spec

# Check if build succeeded
if (Test-Path "dist\q8bot.exe") {
    Write-Host
    Write-Host "========================================" -ForegroundColor Green
    Write-Host "Build successful!" -ForegroundColor Green
    Write-Host "========================================" -ForegroundColor Green
    Write-Host
    Write-Host "Executable location: dist\q8bot.exe" -ForegroundColor Cyan
    Write-Host
    Write-Host "Usage:" -ForegroundColor Yellow
    Write-Host "  .\dist\q8bot.exe           - Auto-detect COM port"
    Write-Host "  .\dist\q8bot.exe COM3      - Use specific COM port"
    Write-Host "  .\dist\q8bot.exe --debug   - Enable debug logging"
    Write-Host
} else {
    Write-Host
    Write-Host "========================================" -ForegroundColor Red
    Write-Host "Build FAILED!" -ForegroundColor Red
    Write-Host "========================================" -ForegroundColor Red
    Write-Host "Please check the output above for errors." -ForegroundColor Yellow
    Write-Host
}

Read-Host "Press Enter to exit"
