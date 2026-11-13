# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Fixed
- Fixed `inverse_kinematics_func` in `simulation/ros/src/my_robot_gazebo/scripts/kinematics.py` to solve 3D position IK via the 3x4 linear Jacobian instead of enforcing zero angular displacement, reducing cold-start convergence error below `1e-4` m.
- Fixed vector orientation handling in `simulation/matlab/kinematics/fk_pos_func.m` and `forward_velocity_kinematics.m`, plus zero-configuration height expectation (`0.42555` m) in MATLAB/Octave test scripts.
- Fixed swapped `transition` and `circle` CSV file mapping, relative `trajectories/` lookup, and gripper safety comment mismatch in `hardware/arduino/send_trajectory.py` and `hardware/arduino/receiver/receiver.ino`.
- Corrected `catkin_install_python` script targets in `simulation/ros/src/my_robot_gazebo/CMakeLists.txt`.
- Aligned IEEE technical report equations, DH table, simulation parameters, benchmark tables, and citations with verified code and literature metadata.

### Added
- Added offline execution, `--dry-run`, `--port`, and `--baud` CLI flags to `hardware/arduino/send_trajectory.py`, and standalone execution mode to `validate_roundtrip.py`.
- Added 25 pytest checks in `tests/test_kinematics_and_trajectories.py`, pinned dependencies via `pyproject.toml` and `uv.lock`, multi-job CI (Python/Ruff/pytest, GNU Octave container, `arduino-cli`), and Dependabot configuration.
- Added `CONTRIBUTING.md`, `CODE_OF_CONDUCT.md`, `SECURITY.md`, `SUPPORT.md`, `CITATION.cff`, and GitHub issue and pull request templates.

## [1.0.0] - 2026-03-18

### Added
- Initial public release covering SolidWorks CAD assembly, Proteus schematic, URDF/Gazebo ROS Noetic package, MATLAB/Simulink Simscape model, 451 trajectory CSV waypoints, Arduino Uno firmware, and technical report.
