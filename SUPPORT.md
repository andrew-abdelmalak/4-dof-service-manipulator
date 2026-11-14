# Getting help

## Documentation and bugs

Read the root [README](README.md), [hardware guide](hardware/arduino/README.md), and [MATLAB/Octave simulation guide](simulation/matlab/README.md) first.

If you encounter a reproducible bug in kinematics, simulation scripts, or firmware, open a report using the [issue forms](https://github.com/andrew-abdelmalak/4-dof-service-manipulator/issues/new/choose).

## Security problems

Follow [SECURITY.md](SECURITY.md) to report vulnerabilities privately.

## What we do not support

- Live ROS Noetic Gazebo physics execution outside Ubuntu 20.04 / ROS Noetic
- Live Simscape `.slx` execution without a licensed MATLAB/Simulink installation
- Custom servo pin mappings or hardware boards other than Arduino Uno without adapting `hardware/arduino/receiver/receiver.ino`
