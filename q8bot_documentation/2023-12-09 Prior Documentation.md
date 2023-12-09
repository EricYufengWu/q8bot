
### Summary
I would like to make a dynamic, quadruped walking robot that is:
- As small as possible
- As simple as possible
- Still capable of dynamic gaits etc., maybe even jumping.

Ever since the MIT Mini Cheetah project, I've seen a lot of attempts of making similar walking robot dog. Many really smart people online have taken their stab at it (myself included, although I'm certainly not in the "very smart" people haha). 

While I've attempted at making QDD actuators and CADing a robot dog design, I wanted to see if I can go the other route and make something that is simple, cheap, tiny, but robust and capable. Thus the project q8bot is born.

As it currently stands, the design is an 8-DOF quadruped robot driven by Dynamixel XL330-M077 motors, similar to the Stanford Doggo robot. This is a smart servo w/ built-in MCU, and although it is not QDD, it has a max speed of 300rpm and enough torque based on my rudimentary calculation.

There will be ZERO WIRES in the final robot design, as all motors will be directly connecting to the central PCB, which will house the MCU, the batteries, and the TTL circuit.

### A Test Stand
First, I'm making a test stand to verify that the motor I selected can actually move the robot. I bought a miniature linear rail and 3D printed the stand. The leg i a simple five-bar parallel mechanism. 
![[IMG_0936.JPEG|400]]
![[IMG_0947.JPEG|400]]

The following shows that the motors was able to make small jumps with the weight of 2 motors, the structure the the weight of the linear rail carrier. Hopefully, with further weight reduction and software improvements, it can jump a little higher. Yay!
![[IMG_0997.mp4]]


### Rev0
The rev0 robot design will have all 8 actuators, and a middle PCB in its intended location, but without all intended features - it will only have the connectors and passive traces, and I'll be using an external TTL driver (e,g. U2D2) to control the robot tethered.

[insert a CAD rendering here]

I designed the structural components of the robot with DFM in mind. The design is largely symmetric. I will only need 3 different types of links for each leg, and the same parts can be repeated 3 more times for the entire robot. 

I will also not be using the original back shell of the Dynamixel robot. Instead, this long, 3D printed piece will act as the motor back shell for all 4 actuators on one side of the robot. The same part is used on the other side and rotated. I designed the fastener geometry so hat they can both hold the screw head and the brass heat-set insert, so that the design is identical, but the post-processing after 3D printing is different for the left and right brackets.

![[IMG_1149.JPEG|400]]
![[IMG_1148.JPEG|400]]
the motor back shell is 1.73g. the bottom image is the assembled middle structure, which replaces 8 motor back shells. The weight is 13.0, which is < 1.73x8=13.84g.

Rev0 PCB:
![[Rev0_PCB.png]]