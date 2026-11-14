# MATLAB, Octave, and Simscape Simulation

Kinematic modeling, task-space and joint-space trajectory generation, and Simscape Multibody files for the 4-DOF service manipulator.

## Directory Layout

```text
simulation/matlab/
├── kinematics/   # DH transforms, FK, 6x4 analytical Jacobian, pinv IK
├── trajectory/   # Square, circular, linear, cubic, and .mat generators
├── simscape/     # Simulink Simscape Multibody assembly (.slx and DataFile.m)
└── tests/        # Verification scripts runnable in MATLAB or GNU Octave
```

## Run Kinematic Verification (GNU Octave or MATLAB)

All five verification scripts run headlessly in GNU Octave 9.2.0 (including inside the pinned Docker container used by CI) as well as MATLAB:

```bash
docker run --rm -v "$PWD:/workspace" -w /workspace \
  gnuoctave/octave:9.2.0 \
  octave --no-gui --quiet --eval "
    set(0, 'defaultfigurevisible', 'off');
    run('simulation/matlab/tests/test_forward_position.m');
    run('simulation/matlab/tests/test_inverse_kinematics.m');
    run('simulation/matlab/tests/test_velocity_kinematics.m');
    run('simulation/matlab/tests/run_fk_4dof.m');
    run('simulation/matlab/kinematics/create_jacobian_func.m');
  "
```

Verified behavior:

- `test_forward_position.m`: confirms `[0, 0, 0, 0]` rad maps to `[0, 0, 0.42555]` m (`0.0000e+00` m error).
- `test_inverse_kinematics.m`: confirms zero-config and cold-start position IK convergence within `1e-3` m (`4.96e-04` m cold-start error).
- `test_velocity_kinematics.m`: confirms velocity roundtrip residuals below `1e-12` rad/s (`1.21e-17` to `2.20e-15` rad/s) via Moore-Penrose `pinv`.
- `run_fk_4dof.m`: checks `forward_kinematics_func.m` against `fk_pos_func.m` (`6.36e-17` m max difference). When MATLAB's Symbolic Math Toolbox is present, it also prints the symbolic position expression.
- `create_jacobian_func.m`: verifies `jacobian_matrix(q)(1:3, :)` against central finite differences (`6.21e-11` max diff without overwriting `jacobian_matrix.m`).

## Core Kinematics Files

| Function | Description |
|---|---|
| `forward_kinematics_func.m` | Coder-compatible FK: `(q1, q2, q3, q4) -> [x; y; z]` |
| `fk_pos_func.m` | Closed-form FK accepting `1x4` or `4x1` input vectors |
| `transformation_func.m` | Standard DH homogeneous transform matrix |
| `jacobian_matrix.m` | 6x4 analytical spatial Jacobian |
| `inverse_jacobian_matrix.m` | Moore-Penrose pseudoinverse `pinv(J)` |
| `inverse_kinematics_func.m` | Position-only Newton-Raphson IK (`pinv`, tol `1e-3` m, `max_iter=100`) |
| `inverse_kinematics_with_orientation.m` | Experimental 6-DOF IK helper |
| `forward_velocity_kinematics.m` | `V = J(q) * q_dot(:)` |
| `inverse_velocity_kinematics.m` | `q_dot = pinv(J(q)) * V_des(:)` |

## Simscape Multibody Model

`simulation/matlab/simscape/x5_DOF_Robot_Assembly.slx` and `x5_DOF_Robot_Assembly_DataFile.m` model the 13 rigid CAD bodies in Simulink/Simscape Multibody. Because `.slx` execution requires a licensed MATLAB/Simulink desktop installation, CI cannot execute `.slx` diagrams directly; the underlying DH transforms, Jacobians, and IK equations are validated independently in Octave and Python.

## DH Parameters

| Link | `theta_i` | `d_i` (m) | `a_i` (m) | `alpha_i` (rad) |
|---:|---|---:|---:|---:|
| 1 | `q1` | 0.04355 | 0 | `pi/2` |
| 2 | `q2 + pi/2` | 0 | 0.140 | `pi` |
| 3 | `q3` | 0 | 0.133 | `pi` |
| 4 | `q4` | 0 | 0.109 | 0 |
