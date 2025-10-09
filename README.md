# Q8bot Quadruped

Q8bot is a miniature quadruped robot with the size and weight comparable to a modern smartphone. This robot is capable of dynamic movements like walking, trotting, bounding, jumping, and more. It also has NO WIRES AND CABLES - everything is directly plugged into the center PCB, greatly reducing complexity, weight, and cost. The current **[Bill of Materials (BOM)](https://docs.google.com/spreadsheets/d/1M1K_Dghia-Mn2t4RStW8juN6r4e3I3OBy6M_fPFHzs8/edit?usp=sharing)**, without optimization, ranges between $300 - $400 depending on the listed options. 

<p align="center">
  <img src="documentation_public/Q8bot_Hero_With_Dimension.jpg" alt="Image 1" width="49%">
  <img src="documentation_public/Q8bot_10s.gif" alt="Image 2" width="49%" style="aspect-ratio: 3/2; object-fit: cover;">
  <img src="documentation_public/Q8bot_Components.jpg" alt="Image 3" width="49%">
  <img src="documentation_public/Q8bot_Weight.jpg" alt="Image 4" width="49%">
</p>

## LATEST UPDATE: 10/09/2025
I am making major updates to the repository's structure to make it more organized and easy to navigate. I appreciate your patience as things may get a bit messy for a short while! Major changes include:
- Move the current main branch to a protected, legacy branch to help with transition
- Retrospectively publishing release bundles containing legacy software and hardware packages for those who may need them.
- Moving hardware source files (STEP, STL, Gerber, etc.) out of the main repository and including them only as part of releases — done to save space and keep Git tracking clean.

In the meantime, I’m also making major updates to the software stack. Please stay tuned for updated firmware and setup instructions!

**For a full history of updates, please go to [UPDATES.md](UPDATES.md)**


## Featured Media & Demos
Q8bot's capabilities: [IROS Video](https://youtu.be/0dk7lYoITQw) 

Detailed building process: [YouTube Video](https://youtu.be/YJDc1xAhaOI)

Q8bot is lucky to be featured in a number of blog posts:
- Hackaday: https://hackaday.com/2024/10/29/little-quadruped-has-pcb-spine-and-no-wiring/#comments
- Hackster.io: https://www.hackster.io/news/a-step-up-for-diy-robotics-8b04a2320861?f=1
- Interesting Engineering: https://interestingengineering.com/innovation/palm-sized-quadruped-robot-redefines-design


## Publications
- Design: https://arxiv.org/abs/2508.01149
- Control & Data Acquisition: https://ieeexplore.ieee.org/abstract/document/11078123/


## Open Source Information
Q8bot is 100% open source: In this repository, you will find everything you need - STEP, STL, Gerber, Schematics, bill of materials (BOM), instructions, and more - to build your own version. 

**You can now ordered the fully-assembled PCB via PCBWay!** This [project page](https://www.pcbway.com/project/shareproject/Q8bot_PCB_Robot_dfa65114.html) contains all of the assembly-related files (BOM, centroid, etc.) and simplifies the ordering process.

<p align="center">
    <a href="https://www.pcbway.com/project/shareproject/Q8bot_PCB_Robot_dfa65114.html"><img src="https://www.pcbway.com/project/img/images/frompcbway-1220.png" alt="PCB from PCBWay" /></a>
</p>

I’m working on turning Q8bot into a purchasable kit, though I’m not sure when or if it will happen. In the meantime, I encourage you to build your own Q8bot! This palm-sized quadruped is perfect for robotics education, swarm robotics research, or simply as a fun engineering project. If you have any questions, feel free to reach out and I will try my best to help. **If you have built your own Q8bot, please consider sending photos or videos to me! I would love to feature your creation here publically with your permission!**

Personal: yufeng.wu0902@gmail.com

School: ericyufengwu@ucla.edu


## Building Instruction
The building instructions have moved to a dedicated folder:

[Sourcing Components](building_instructions/sourcing_components.md)

[Assembling the Robot](building_instructions/robot_assembly.md)

[Software Setup](building_instructions/software_setup.md)


## Note on Project Ownership
Q8bot is currently considered as a project at the [UCLA Robotics and Mechanisms Lab (RoMeLa)](https://www.romela.org/). I am grateful for the support from Dr. Dennis Hong and fellow RoMeLa members in helping publish my work as an academic research paper.

That said, since Q8bot was originally conceived and developed independently prior to my time at UCLA, I will retain full ownership of the original design after graduation. Given the project’s open-source nature, RoMeLa will have the freedom to continue developing, modifying, and publishing research based on the platform. A derived successor design is currently under development and will be owned by RoMeLa.