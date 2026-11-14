# CAD Files

SolidWorks CAD design files that we created for the 4-DOF service robotic manipulator.

## Assembly

- **`5_DOF_Robot_Assembly.SLDASM`**: Full SolidWorks assembly (requires SolidWorks 2020+)

## Part Files (STEP)

Individual part exports in STEP format (universal, compatible with any CAD software):

| Part | Description |
|------|-------------|
| Base Link | Stationary base housing the MG996R base servo |
| Arm 1 (Shoulder) | First link driven by the shoulder servo |
| Arm 2 (Elbow) | Second link driven by the elbow servo |
| Arm 3 (Wrist) | Wrist link driven by SG90 micro servo |
| Gripper | Parallel-jaw gripper with friction-lined inner surfaces |
| Rotating Waste | Fixed pivot component (maintains 4-DOF kinematic chain) |

## Manufacturing

- **Material:** PLA (3D-printed on FDM printer)
- **Servos:** 3× MG996R (base, shoulder, elbow) + 3× SG90 (wrist, gripper, fixed pivot)
- **Assembly:** Servo horns press-fit into 3D-printed brackets; base secured to weighted wooden board

## Mesh Exports

STL mesh files that we exported for Gazebo/RViz visualization live at:

```text
simulation/ros/src/my_robot_gazebo/meshes/
```
