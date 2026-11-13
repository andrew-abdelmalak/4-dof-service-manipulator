# Contributing to 4-dof-service-manipulator

Bug reports, kinematics or trajectory test fixes, and documentation improvements are welcome. For larger changes, open an issue first to discuss the design.

## Before you start

- Search [existing issues](https://github.com/andrew-abdelmalak/4-dof-service-manipulator/issues) to avoid duplicates.
- Open an issue before starting work on behavioral changes to kinematics, trajectory generation, or firmware protocol.
- By participating, you agree to follow our [Code of Conduct](CODE_OF_CONDUCT.md).

## Set up your environment

Requirements: Python 3.11+ with `uv`, Docker (for GNU Octave 9.2.0 and TeX Live), and `arduino-cli` (`arduino:avr` core) for firmware compilation checks.

```bash
git clone https://github.com/andrew-abdelmalak/4-dof-service-manipulator.git
cd 4-dof-service-manipulator
uv sync --locked
```

## Run the checks

```bash
uv run ruff check .
uv run ruff format --check tests/ hardware/arduino/send_trajectory.py simulation/ros/src/my_robot_gazebo/scripts/kinematics.py simulation/ros/src/my_robot_gazebo/scripts/validate_roundtrip.py
uv run pytest -v
uv run python simulation/ros/src/my_robot_gazebo/scripts/validate_roundtrip.py
uv run python hardware/arduino/send_trajectory.py --dry-run
arduino-cli compile --warnings all --fqbn arduino:avr:uno hardware/arduino/receiver/receiver.ino
```

To run the MATLAB/Octave validation suite locally with Docker:

```bash
docker run --rm -v "$PWD:/workspace" -w /workspace \
  gnuoctave/octave:9.2.0 \
  octave --no-gui --quiet --eval "
    run('simulation/matlab/tests/test_forward_position.m');
    run('simulation/matlab/tests/test_inverse_kinematics.m');
    run('simulation/matlab/tests/test_velocity_kinematics.m');
    run('simulation/matlab/tests/run_fk_4dof.m');
    run('simulation/matlab/kinematics/create_jacobian_func.m');
  "
```

CI runs the same Python, Octave, and Arduino compile checks on every push and pull request. Pull requests with failing checks cannot be merged.

## Make your change

1. Create a branch from `main`: `git switch -c fix/short-description`.
2. Keep each pull request focused on a single topic.
3. Add or update unit tests in `tests/` when changing kinematics, planners, or CLI parsers.
4. Keep the technical report (`docs/4DOF_Service_Manipulator.pdf`) and README tables consistent with code changes.

## Commit messages

Use a concise imperative summary under 72 characters (for example, `Fix position IK Jacobian rows for 4-DOF solver`).

## Open a pull request

1. Push your branch and open a pull request against `main`.
2. Complete the pull request template and reference any related issue (`Fixes #123`).
3. Ensure all required status checks pass.

## Report a bug or request a feature

Use the [issue forms](https://github.com/andrew-abdelmalak/4-dof-service-manipulator/issues/new/choose). For security issues, follow [SECURITY.md](SECURITY.md) instead of opening a public issue.

## License

By contributing, you agree that your contributions are licensed under the project's [MIT License](LICENSE).
