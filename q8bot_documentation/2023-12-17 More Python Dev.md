### Near-term To dos:
1. ~~Try out bulk read and bulk write in DynamixelSDK.~~
2. Wrap DynamixelSDK into a class (easily do timed move, instant move, etc).
3. Write q8_operate.py function to control a single leg by keyboard input.
    a) End effector location.
    b) Motor angles.
    c) Upon user input, print angle and position values.
4. Multiply the function by 4 to control static position of the whole robot.
5. Look into gait generation.
6. ...


I implemented bulk write protocol in the leg jumping test code, and it noticeably synced up the two motors and improved the jumping performance a little bit. Nice.
![[IMG_0997.mp4]]
After:
![[IMG_1217.mp4]]