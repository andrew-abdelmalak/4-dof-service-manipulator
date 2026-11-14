# my_robot_gazebo (ROS Noetic + Gazebo Classic)

We built the `my_robot_gazebo` catkin package to model, visualize, and simulate our 4-DOF service robotic manipulator in ROS Noetic, RViz, and Gazebo Classic, while also exposing standalone offline kinematics and trajectory scripts.

## Package Layout

```text
my_robot_gazebo/
├── config/controllers.yaml     # JointStateController and per-joint PID parameters
├── launch/
│   ├── spawn_gazebo.launch     # Gazebo world, URDF spawn, kinematic driver, and EE pose publisher
│   └── view_model.launch       # RViz visualization with joint_state_publisher_gui
├── meshes/                     # Binary STL meshes referenced by urdf/robot.urdf
├── models/my_robot/            # Gazebo model configuration and SDF metadata
├── rviz/model_view.rviz        # Preconfigured RViz display settings
├── scripts/                    # Kinematics, planners, ROS validation nodes, and offline checks
├── urdf/robot.urdf             # 5-joint URDF with fixed Joint_4 to enforce the 4-DOF chain
├── worlds/stable.world         # Gazebo simulation world
├── CMakeLists.txt
└── package.xml
```

## Build in a Catkin Workspace

Link or copy this package into a ROS Noetic workspace and build with `catkin_make`:

```bash
mkdir -p ~/catkin_ws/src
ln -s /path/to/4-dof-service-manipulator/simulation/ros/src/my_robot_gazebo ~/catkin_ws/src/my_robot_gazebo
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Launch Simulation and Visualization

- **Gazebo simulation:** Starts `worlds/stable.world`, spawns `urdf/robot.urdf` as `my_robot`, launches `kinematic_joint_driver.py` (30 Hz), and publishes `end_effector_pose` from `/gazebo/get_link_state` for `my_robot::EE_frame`:

```bash
roslaunch my_robot_gazebo spawn_gazebo.launch
```

- **RViz model check (no Gazebo):** Opens `urdf/robot.urdf` with interactive joint sliders:

```bash
roslaunch my_robot_gazebo view_model.launch
```

- **Trajectory execution:** Streams the five tea-preparation phases (`square1`, `square2`, `square2_rev`, `transition`, `circle`):

```bash
rosrun my_robot_gazebo trajectory_node.py
```

## Offline Python Checks (No ROS Installation Required)

We decoupled the kinematics and roundtrip checks from `rospy` so they run directly with `uv` on any host:

```bash
uv run python simulation/ros/src/my_robot_gazebo/scripts/forward_kinematics.py
uv run python simulation/ros/src/my_robot_gazebo/scripts/validate_roundtrip.py
```
