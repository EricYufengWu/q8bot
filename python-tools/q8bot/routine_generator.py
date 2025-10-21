'''
Written by yufeng.wu0902@gmail.com

Routine generation module for Q8bot scripted movements.
This module contains pre-defined via-point sequences and choreographed routines
for non-locomotion movements like greetings, range demonstrations, etc.

To add to this module, define a set of via points following the existing format,
and create your own function that moves through the points in your desired 
manner.
'''

import time

# Q8bot range of motion via points
R1 = [100, 80, 100, 80, 100, 80, 100, 80]
R2 = [0, 45, 0, 45, 0, 45, 0, 45]
R3 = [-90, 45, -90, 45, -90, 45, -90, 45]
R4 = [-20, 200, -20, 200, -20, 200, -20, 200]
R5 = [130, 270, 130, 270, 130, 270, 130, 270]
R6 = [130, 180, 130, 180, 130, 180, 130, 180]
R7 = [100, 80, 100, 80, 100, 80, 100, 80]
R8 = [-20, 200, -20, 200, -20, 200, -20, 200]
R9 = [30, 150, 30, 150, 30, 150, 30, 150]
R = [R1, R2, R3, R4, R5, R6, R7, R8, R9]

# Q8bot greet motion via points
G1 = [-90, 45, -90, 45, -90, 45, -90, 45]
G2 = [0, 45, 0, 45, 0, 45, 0, 45]
G3 = [-45, 45, -45, 45, 50, 75, 50, 75]
G4 = [45, 90, -45, 45, 50, 75, 50, 75]
G5 = [-45, 45, 45, 90, 50, 75, 50, 75]
G6 = [-90, 45, -90, 45, -90, 45, -90, 45]


def show_range(q8):
    for pos in R:
        q8.move_all(pos, 1000, False)
        time.sleep(1.5)
    return


def greet(q8):
    q8.move_all(G1, 1000, False)
    time.sleep(1.1)
    q8.move_all(G2, 1000, False)
    time.sleep(1)
    q8.move_all(G3, 500, False)
    time.sleep(1)
    for _ in range(3):
        q8.move_all(G4, 200, False)
        time.sleep(0.25)
        q8.move_all(G5, 200, False)
        time.sleep(0.25)
    time.sleep(0.75)
    q8.move_all(G6, 500, False)
    time.sleep(0.7)
