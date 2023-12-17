I made an Jupyter Notebook doc that performs forward kinematics calculation on the parallel five-bar mechanism. After working out the correct solution, I also optimized the code to make it fast enough to run in real time.

![[FK_diagram_1.jpg|360]]![[FK_diagram_2.jpg|540]]

The code still has a minor bug: switching to the other solution (concave down) at some location, sporadically. I will find time to address this later.

Luckily, the IK is much easier - only requires the law of cosines equation.
![[IK_diagram_1.jpg|360]]
![[IK_diagram_2.jpg|360]]