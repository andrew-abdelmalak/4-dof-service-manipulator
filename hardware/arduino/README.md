# Arduino Firmware and Serial Trajectory Controller

Open-loop trajectory execution controller for the 4-DOF service manipulator on an Arduino Uno (`receiver/receiver.ino`) driven over USB serial by `send_trajectory.py`.

## Hardware Mapping and Firmware Constants

- **Board:** Arduino Uno (`arduino:avr:uno`)
- **Baud rate:** 115200
- **Servos attached in firmware (`receiver.ino`):** five `Servo` instances on PWM pins `3` (base J1), `5` (shoulder J2), `6` (elbow J3), `9` (wrist J4), and `10` (gripper). One additional physical SG90 servo acts as a fixed mechanical pivot in the 4-DOF arm assembly.
- **Gripper limits:** `GRIPPER_OPEN = 90.0` deg (servo 180 deg), `GRIPPER_CLOSED = 30.0` deg (servo 120 deg), `GRIPPER_MIN_SAFE = 30.0` deg safety floor.

## Trajectory CSV Files

All five CSV files live in `trajectories/` and contain 4 columns (`J1, J2, J3, J4` in degrees). Header rows are skipped automatically if present.

| Phase | CSV File | Waypoints | Motion |
|---|---|---:|---|
| 1 | `trajectory_data_combined_1_square1.csv` | 103 | Sugar pick-and-place square path |
| 2 | `trajectory_data_combined_2_square2.csv` | 103 | Spoon approach square path |
| 3 | `trajectory_data_combined_3_square2_rev.csv` | 103 | Return to cup along reverse square path |
| 4 | `trajectory_data_combined_4_transition.csv` | 41 | Linear transition into stirring posture |
| 5 | `trajectory_data_combined_5_circle.csv` | 101 | Circular stirring loop (executed 3x) |

Total unique waypoints: **451** (653 streamed commands during a full 3-cycle stirring run).

## Compile Firmware Without Hardware

Verify the firmware compiles cleanly using `arduino-cli`:

```bash
arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli lib install Servo
arduino-cli compile --warnings all --fqbn arduino:avr:uno hardware/arduino/receiver/receiver.ino
```

## Validate Trajectory Files Offline

Run `send_trajectory.py` in `--dry-run` mode from any working directory to load, parse, and summarize all 451 waypoints without opening a serial port:

```bash
uv run python hardware/arduino/send_trajectory.py --dry-run
```

## Execute on Physical Hardware

1. Flash `hardware/arduino/receiver/receiver.ino` to the Arduino Uno (`arduino-cli upload` or Arduino IDE).
2. Connect the 12 V supply (regulated down for servo rails) with common ground to the Arduino.
3. Run the sender script against your serial port:

```bash
uv run python hardware/arduino/send_trajectory.py --port /dev/ttyACM0 --baud 115200
```

Replace `/dev/ttyACM0` with `/dev/cu.usbserial-110` on macOS or `COM3` on Windows.

## Serial Protocol

- **Joint command:** `J:<j1>,<j2>,<j3>,<j4>\n` (angles in degrees; firmware maps mechanical offsets and clamps to `[0, 180]` deg servo range).
- **Gripper commands:** `GRIPPER_OPEN\n` and `GRIPPER_CLOSE\n`.
- **Handshake:** Arduino responds with `ARDUINO_CONNECTED` on boot and `READY` after executing each line.
