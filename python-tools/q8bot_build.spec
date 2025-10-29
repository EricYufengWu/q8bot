# -*- mode: python ; coding: utf-8 -*-
"""
PyInstaller spec file for Q8bot single-file executable.

Usage:
    pyinstaller q8bot_build.spec

Output:
    dist/q8bot.exe - Single file executable
"""

block_cipher = None

a = Analysis(
    ['q8bot/operate.py'],
    pathex=[],
    binaries=[],
    datas=[
        ('docs/Instruction_Default.jpg', 'docs'),
        ('docs/Instruction_Joystick.jpg', 'docs'),
    ],
    hiddenimports=[
        'numpy',
        'scipy',
        'scipy.optimize',
        'pygame',
        'serial',
        'serial.tools',
        'serial.tools.list_ports',
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[
        'matplotlib',  # Exclude archived graphing library
        'PIL',
        'pillow',
        'sympy',
    ],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.zipfiles,
    a.datas,
    [],
    name='q8bot',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=True,  # Show console for logging output
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=None,  # Add icon path here if you have one
)
