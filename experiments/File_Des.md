# File descriptions

- Files from original Taxim: `grasp_air_video.py`, `grasp_data_video.py`.

- Files modified from original Taxim: `grasp_air_demo.py`, `grasp_data_demo.py`.
  - These files use original UR5e robot and wsg50 gripper. The modified version includes more objects to observe changes in tactile signal.

- File contains multiple object for object interaction observation:  `grasp_mult_demo.py `
  - This file uses multiple objects and simulate the collision of two objects.

- File using Franka panda robot:  `grasp_multi_panda.py `
  - This file uses Franka robot and updated version of GelSight for better correspondance with real environment. 
  - Can be modified to multiple object interaction simulation.
  - TODO: currently having problem with appropriate GelSight signals. Will try to look more into the issues.


