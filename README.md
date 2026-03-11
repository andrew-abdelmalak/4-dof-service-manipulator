# 4-DOF Service Robotic Manipulator for Automated Beverage Preparation

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![ROS](https://img.shields.io/badge/ROS-Noetic-blue)](http://wiki.ros.org/noetic)
[![MATLAB](https://img.shields.io/badge/MATLAB-R2023b-orange)](https://www.mathworks.com/products/matlab.html)
[![Arduino](https://img.shields.io/badge/Arduino-Uno-teal)](https://www.arduino.cc/)

This repository contains the source code, simulation environments, and hardware control software accompanying the paper:

> **Design, Kinematic Analysis, and Experimental Validation of a 4-DOF Service Robotic Manipulator for Automated Beverage Preparation**
>
> A. Abdelmalak, D. Boules, D. Girgis, K. Kirolous, S. Yacoub, Y. Salama — *German University in Cairo (GUC)*

---

## Overview

A desktop-scale **4-DOF service manipulator** (5 joints, Joint 4 fixed) built from 3D-printed PLA and actuated by MG996R + SG90 servos. The system performs automated tea preparation: sugar transport, spoon manipulation, and stirring.

### DH Parameters

| Link | θᵢ | dᵢ (m) | aᵢ (m) | αᵢ (rad) |
|------|-----|---------|---------|----------|
| 1 | q₁ + π/2 | 0.04355 | 0 | π/2 |
| 2 | q₂ | 0 | 0.140 | π |
| 3 | q₃ | 0 | 0.133 | π |
| 4 | q₄ | 0 | 0.109 | 0 |

### Key Results

| Metric | Value |
|--------|-------|
| FK roundtrip positional error | 0.000 m |
| IK roundtrip velocity error | ~10⁻¹⁵ (machine precision) |
| Zero-config end-effector height | 0.4255 m |
| Trajectory sampling period | 0.1 s |
| IK damping factor (λ) | 0.05 |

---

## Repository Structure

```
├── simulation/
│   ├── ros/src/                        # ROS/Gazebo simulation
│   │   └── my_robot_gazebo/            # ROS package
│   │       ├── urdf/robot.urdf         # Robot description
│   │       ├── launch/                 # Gazebo launch files
│   │       ├── meshes/                 # STL mesh files
│   │       ├── config/                 # Controller configs
│   │       ├── scripts/                # Python nodes (FK, IK, trajectory)
│   │       ├── models/                 # Gazebo world models
│   │       └── worlds/                 # Gazebo world files
│   │
│   └── matlab/                         # MATLAB/Simulink
│       ├── kinematics/                 # FK, IK, Jacobian functions
│       ├── trajectory/                 # Trajectory planners + CSV data
│       ├── simscape/                   # Simulink Simscape Multibody model
│       └── tests/                      # Validation scripts
│
├── hardware/
│   ├── arduino/                        # Arduino servo control
│   │   ├── receiver/receiver.ino       # Arduino firmware
│   │   ├── send_trajectory.py          # PC → Arduino trajectory sender
│   │   └── trajectories/              # Pre-computed CSV joint angles
│   │
│   └── cad/                            # SolidWorks CAD exports
│       ├── 5_DOF_Robot_Assembly.SLDASM # Full assembly
│       └── *.STEP                      # Individual part files
│
├── .gitignore
├── LICENSE
└── README.md
```

---

## Getting Started

### Prerequisites

| Tool | Version | Purpose |
|------|---------|---------|
| ROS Noetic | Ubuntu 20.04 | Gazebo simulation |
| MATLAB | R2023b+ | Kinematics + Simscape |
| Python | 3.8+ | ROS nodes + Arduino comm |
| Arduino IDE | 2.0+ | Firmware upload |

### ROS / Gazebo

```bash
mkdir -p ~/catkin_ws/src
ln -s /path/to/this/repo/simulation/ros/src/my_robot_gazebo ~/catkin_ws/src/
cd ~/catkin_ws && catkin_make && source devel/setup.bash

# Launch robot in Gazebo
roslaunch my_robot_gazebo spawn_gazebo.launch

# Run trajectory execution (separate terminal)
rosrun my_robot_gazebo trajectory_node.py
```

### MATLAB / Simulink

```matlab
addpath('simulation/matlab/kinematics');
addpath('simulation/matlab/trajectory');
addpath('simulation/matlab/tests');

test_forward_position     % FK validation
test_inverse_kinematics   % IK validation
test_velocity_kinematics  % Jacobian validation

% Open Simscape model
open('simulation/matlab/simscape/x5_DOF_Robot_Assembly.slx')
```

### Arduino Hardware

1. Upload `hardware/arduino/receiver/receiver.ino` to the Arduino Uno
2. Connect servos per pin assignments in the paper (Table II)
3. Run: `python hardware/arduino/send_trajectory.py`

---

## Citation

```bibtex
@inproceedings{abdelmalak2025manipulator,
  title   = {Design, Kinematic Analysis, and Experimental Validation of a 
             4-DOF Service Robotic Manipulator for Automated Beverage Preparation},
  author  = {Abdelmalak, Andrew and Boules, Daniel and Girgis, David and 
             Kirolous, Kirolous and Yacoub, Samir and Salama, Youssef},
  year    = {2025},
  school  = {German University in Cairo}
}
```

## Authors

| Name | Affiliation |
|------|-------------|
| **Andrew Abdelmalak** | Mechatronics Engineering, GUC |
| **Daniel Boules** | Mechatronics Engineering, GUC |
| **David Girgis** | Mechatronics Engineering, GUC |
| **Kirolous Kirolous** | Mechatronics Engineering, GUC |
| **Samir Yacoub** | Mechatronics Engineering, GUC |
| **Youssef Salama** | Mechatronics Engineering, GUC |

## License

MIT — see [LICENSE](LICENSE).