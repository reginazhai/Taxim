# Taxim-panda: 

This is a modified version of Taxim, which replaces UR5e robot with Franka robot


## Installation

You can manually clone the repository and install the package using:

```bash
git clone -b taxim-robot https://github.com/CMURoboTouch/Taxim
cd Taxim
pip install -e .
```

To install dependencies:

```bash
pip install -r requirements.txt
```

## Content

This package contain several components:

1) A renderer to simulate tactile readings from a GelSight.
2) A contact model maps normal forces to indentation depths and shear forces to shear displacements.
2) Mesh models and urdf files for GelSight, UR5e robot, WSG50 gripper, Franka Emika robot and objects.
3) Examples of grasping.

## Usage

### experiments

```bash
python grasp_multi_panda.py -bj RubiksCube
```

`grasp_multi_panda.py` is used to demonstrate grasping on objects that can stand on the table using. 

## Operating System

We recommend to conduct experiments on **Ubuntu**.

For **macOS**, there exists some visualization problem between pybullet.GUI and pyrender as we know of. Please let us
know if it can be resolved, and we will share the information at the repo!

## License
This project is licensed under MIT license, as found in the [LICENSE](LICENSE) file.


