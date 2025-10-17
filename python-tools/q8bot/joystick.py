import pygame, time

deadzone = 0.01
def dz(x, dz = deadzone):
    return 0.0 if abs(x) < dz else x

pygame.init()
pygame.joystick.init()
js = pygame.joystick.Joystick(0)
js.init()

while True:
    pygame.event.pump()

    axes = [round(dz(js.get_axis(i)), 2) for i in range(js.get_numaxes())]
    btns = [js.get_button(i) for i in range(js.get_numbuttons())]
    hats = [js.get_hat(i) for i in range(js.get_numhats())]

    print(f"axes: {axes}, buttons {btns}, hats: {hats}")

    time.sleep(0.05)