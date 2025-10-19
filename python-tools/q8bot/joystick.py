# Nintendo Switch Joycon R mapping:
# Official name: 'Nintendo Switch Joy-Con (R)'
# Axes[0] - move right 1.0, move left -1.0, analog, deadzone 0.15
# Axes[1] - move down 1.0, move up -1.0, analog, deadzone 0.15
# Buttons - top: [2], right: [0], down: [1], left: [3], side 1: [16], side 2: [18]

# GPDWIN controller mapping:
# Official name: 'GPD WIN Game Controller'
# Axes[0] - move right 1.0, move left -1.0, analog, deadzone 0.1
# Axes[1] - move down 1.0, move up -1.0, analog, deadzone 0.1
# Buttons - top: [0], right: [1], down: [2], left: [3], side 1: [5], side 2: [4]


import pygame, time, sys

deadzone = 0.01
def dz(x, dz = deadzone):
    return 0.0 if abs(x) < dz else x

pygame.init()
pygame.joystick.init()

count = pygame.joystick.get_count()
if count == 0:
    print("No joystick connected")
    sys.exit(1)

# Print names of all connected joysticks
for i in range(count):
    j = pygame.joystick.Joystick(i)
    j.init()
    print(f"Joystick {i}: {j.get_name()}")

# Use the first joystick as before
js = pygame.joystick.Joystick(0)
js.init()
print(f"Using joystick 0: {js.get_name()}")

while True:
    pygame.event.pump()

    axes = [round(dz(js.get_axis(i)), 2) for i in range(js.get_numaxes())]
    btns = [js.get_button(i) for i in range(js.get_numbuttons())]
    hats = [js.get_hat(i) for i in range(js.get_numhats())]

    print(f"axes: {axes}, buttons {btns}, hats: {hats}")

    time.sleep(0.05)