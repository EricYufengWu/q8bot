import time
import os
from kinematics_solver import *
from q8_dynamixel import *
import pygame

speed = 80
speed_min = 20
speed_max = 300
res = 0.5
y_min = 15

# Jumping parameters
JUMP_LOW = [11, 169, 11, 169, 11, 169, 11, 169] 
JUMP_HIGH = [95, 85, 95, 85, 95, 85, 95, 85]
JUMP_REST = [30, 150, 30, 150, 30, 150, 30, 150]

def jump():
    q8.move_all(JUMP_REST, 700)
    time.sleep(0.3)
    q8.move_all(JUMP_LOW, 500)
    time.sleep(0.5)
    q8.move_all(JUMP_HIGH, 0)
    time.sleep(0.1)
    q8.move_all(JUMP_REST, 0)
    time.sleep(0.5)
    return

pygame.init()
window = pygame.display.set_mode((300, 300))
clock = pygame.time.Clock()

leg = k_solver()
q8 = q8_dynamixel('COM3')
q8.enable_torque()

pos_x = leg.d/2
pos_y = (leg.l1 + leg.l2) * 0.75
q1, q2, success = leg.ik_solve(pos_x, pos_y)
cmd_pos = []
for i in range(4):
    cmd_pos.append(q1)
    cmd_pos.append(q2)
q8.move_all(cmd_pos, 1000)
time.sleep(2)

while True:
    clock.tick(speed)
    pygame.event.get()
    keys = pygame.key.get_pressed()
    if keys[pygame.K_UP] and speed < speed_max:
        speed += 5
        print('+speed')
        time.sleep(0.2)
    elif keys[pygame.K_DOWN] and speed > speed_min:
        speed -= 5
        print('-speed')
        time.sleep(0.2)
    elif keys[pygame.K_b]:
        voltage = q8.check_voltage()
        print("Battery Voltage: %.1f" % (voltage))
        time.sleep(0.2)
    elif keys[pygame.K_j]:
        print("Jumping...")
        jump()
        pos_x, pos_y = 9.75, 28  # JUMP_REST position
        continue
    elif keys[pygame.K_ESCAPE]:
        break
    
    temp_x = pos_x + (keys[pygame.K_d] - keys[pygame.K_a]) * res
    temp_y = pos_y + (keys[pygame.K_w] - keys[pygame.K_s]) * res
    q1, q2, success = leg.ik_solve(temp_x, temp_y)
    if success and temp_y > y_min:
        cmd_pos = []
        for i in range(4):
            cmd_pos.append(q1)
            cmd_pos.append(q2)
        q8.move_all(cmd_pos, 0)
        pos_x, pos_y = temp_x, temp_y

    # print(pos_x, pos_y)

q8.disable_torque()
pygame.quit()
exit()