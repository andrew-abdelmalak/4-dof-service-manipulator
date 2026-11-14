#!/usr/bin/env python3
"""Serial trajectory sender for the 4-DOF service manipulator Arduino controller."""

import argparse
import csv
import sys
import time
from pathlib import Path

# ================= CONFIGURATION =================
DEFAULT_SERIAL_PORT = "/dev/cu.usbserial-110"  # Linux: /dev/ttyACM0 or /dev/ttyUSB0; Windows: COM3
DEFAULT_BAUD_RATE = 115200  # Must match receiver.ino
TRAJECTORIES_DIR = Path(__file__).resolve().parent / "trajectories"

CSV_FILE_1 = TRAJECTORIES_DIR / "trajectory_data_combined_1_square1.csv"
CSV_FILE_2 = TRAJECTORIES_DIR / "trajectory_data_combined_2_square2.csv"
CSV_FILE_3 = TRAJECTORIES_DIR / "trajectory_data_combined_3_square2_rev.csv"
CSV_FILE_4 = TRAJECTORIES_DIR / "trajectory_data_combined_4_transition.csv"
CSV_FILE_5 = TRAJECTORIES_DIR / "trajectory_data_combined_5_circle.csv"

DELAY_BETWEEN_POINTS = 0.1  # Seconds (matches 100 ms trajectory step)
CIRCLE_DELAY = 0.05  # Faster delay for stirring circle trajectory (50 ms)

# Gripper Control
GRIPPER_OPEN_CMD = "GRIPPER_OPEN\n"
GRIPPER_CLOSE_CMD = "GRIPPER_CLOSE\n"
# =================================================


def load_trajectory_csv(csv_path: Path) -> list[list[float]]:
    """Load 4-DOF joint angles (degrees) from a CSV file, skipping optional header rows."""
    waypoints: list[list[float]] = []
    with Path(csv_path).open(newline="", encoding="utf-8") as handle:
        for row_num, row in enumerate(csv.reader(handle), start=1):
            if not row or len(row) < 4:
                continue
            try:
                vals = [float(x) for x in row[:4]]
            except ValueError:
                if row_num == 1:
                    continue
                raise ValueError(f"Invalid numeric row {row_num} in {csv_path}: {row}") from None
            waypoints.append(vals)
    if not waypoints:
        raise ValueError(f"Trajectory CSV has no valid waypoints: {csv_path}")
    return waypoints


def load_all_trajectories() -> dict[str, list[list[float]]]:
    """Load and validate all five sequential phases."""
    phases = {
        "square1": load_trajectory_csv(CSV_FILE_1),
        "square2": load_trajectory_csv(CSV_FILE_2),
        "square2_rev": load_trajectory_csv(CSV_FILE_3),
        "transition": load_trajectory_csv(CSV_FILE_4),
        "circle": load_trajectory_csv(CSV_FILE_5),
    }
    return phases


def send_gripper_command(ser, command_name: str, command: str) -> None:
    """Send a gripper command and wait for READY acknowledgment."""
    print(f"\n>>> {command_name}...")
    ser.write(command.encode("utf-8"))

    while True:
        response = ser.readline().decode("utf-8", errors="replace").strip()
        if response:
            print(f"    Arduino: {response}")
        if "READY" in response:
            break


def send_trajectory_point(
    ser, vals: list[float], point_num: int, total: int, delay: float | None = None
) -> None:
    """Send a single joint-angle waypoint and wait for READY acknowledgment."""
    if delay is None:
        delay = DELAY_BETWEEN_POINTS

    msg = f"{vals[0]},{vals[1]},{vals[2]},{vals[3]}\n"
    ser.write(msg.encode("utf-8"))

    sys.stdout.write(f"\rSending Point {point_num}/{total}: {msg.strip()}   ")
    sys.stdout.flush()

    while True:
        response = ser.readline().decode("utf-8", errors="replace").strip()
        if "READY" in response:
            break

    time.sleep(delay)


def run_dry_run(phases: dict[str, list[list[float]]]) -> None:
    """Validate all CSV files and print the exact serial execution summary."""
    unique_waypoints = sum(len(rows) for rows in phases.values())
    streamed_points = (
        len(phases["square1"])
        + len(phases["square2"])
        + len(phases["square2_rev"])
        + len(phases["transition"])
        + 3 * len(phases["circle"])
    )
    print("=" * 50)
    print("DRY RUN: TRAJECTORY VALIDATION SUMMARY")
    print("=" * 50)
    for name, rows in phases.items():
        first = ", ".join(f"{v:.2f}" for v in rows[0])
        last = ", ".join(f"{v:.2f}" for v in rows[-1])
        print(f"  {name:12s}: {len(rows):3d} waypoints | first=[{first}] | last=[{last}]")
    print(f"Total unique CSV waypoints: {unique_waypoints}")
    print(f"Total streamed points (with 3x circle): {streamed_points}")
    print("Sequence preview:")
    print(f"  1. {GRIPPER_OPEN_CMD.strip()}")
    q0 = phases["square1"][0]
    print(f"  2. {q0[0]:.2f},{q0[1]:.2f},{q0[2]:.2f},{q0[3]:.2f}")
    print(f"  3. {GRIPPER_CLOSE_CMD.strip()}")
    print(
        f"  4. Stream remaining {len(phases['square1']) - 1} square1 points -> "
        f"{GRIPPER_OPEN_CMD.strip()}"
    )
    print(f"  5. Stream {len(phases['square2'])} square2 points -> {GRIPPER_CLOSE_CMD.strip()}")
    print(f"  6. Stream {len(phases['square2_rev'])} square2_rev points")
    print(f"  7. Stream {len(phases['transition'])} transition points")
    print(f"  8. Stream {len(phases['circle'])} circle points x 3 cycles")


def execute_serial_sequence(port: str, baud: int, phases: dict[str, list[list[float]]]) -> None:
    """Stream all five trajectory phases to the Arduino controller over serial."""
    import serial

    ser = serial.Serial(port, baud, timeout=1)
    try:
        time.sleep(2)  # Wait for Arduino reset on serial open
        print(f"Connected to {port} at {baud} baud")

        print("Waiting for Arduino boot...")
        while True:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if "ARDUINO_CONNECTED" in line:
                print("Arduino is online!")
                break

        square1_data = phases["square1"]
        square2_data = phases["square2"]
        square2_reverse_data = phases["square2_rev"]
        transition_data = phases["transition"]
        circle_data = phases["circle"]

        # SQUARE 1: PICK AND MOVE
        print("\n" + "=" * 50)
        print("EXECUTING SQUARE 1: PICK AND MOVE")
        print("=" * 50)
        send_gripper_command(ser, "Opening Gripper", GRIPPER_OPEN_CMD)

        print("\n>>> Moving to pickup position (first point)...")
        send_trajectory_point(ser, square1_data[0], 1, len(square1_data))

        print("\nWaiting for robot to stabilize...")
        time.sleep(1.0)

        send_gripper_command(ser, "Closing Gripper to Grab Item", GRIPPER_CLOSE_CMD)

        print("\n>>> Executing Square 1 trajectory with item...")
        for i, vals in enumerate(square1_data[1:], start=2):
            send_trajectory_point(ser, vals, i, len(square1_data))

        print("\n")
        send_gripper_command(ser, "Opening Gripper to Release Item", GRIPPER_OPEN_CMD)

        # SQUARE 2: MOVE AND PLACE
        print("\n" + "=" * 50)
        print("EXECUTING SQUARE 2: MOVE AND PLACE")
        print("=" * 50)
        print("\n>>> Executing Square 2 trajectory with gripper open...")
        for i, vals in enumerate(square2_data, start=1):
            send_trajectory_point(ser, vals, i, len(square2_data))

        print("\nWaiting for robot to stabilize at placement position...")
        time.sleep(1.0)

        print("\n")
        send_gripper_command(ser, "Closing Gripper to Place Item", GRIPPER_CLOSE_CMD)

        # SQUARE 2 REVERSE: RETURN PATH
        print("\n" + "=" * 50)
        print("EXECUTING SQUARE 2 REVERSE: RETURN PATH")
        print("=" * 50)
        print("\n>>> Executing Square 2 Reverse trajectory (gripper stays closed)...")
        for i, vals in enumerate(square2_reverse_data, start=1):
            send_trajectory_point(ser, vals, i, len(square2_reverse_data))

        # TRANSITION: PREPARE FOR CIRCLE
        print("\n" + "=" * 50)
        print("EXECUTING TRANSITION TRAJECTORY")
        print("=" * 50)
        print("\n>>> Executing Transition trajectory (gripper stays closed)...")
        for i, vals in enumerate(transition_data, start=1):
            send_trajectory_point(ser, vals, i, len(transition_data))

        # CIRCLE: STIRRING (3 CYCLES)
        print("\n" + "=" * 50)
        print("EXECUTING CIRCLE TRAJECTORY (3 REPETITIONS)")
        print("=" * 50)
        for circle_num in range(1, 4):
            print(
                f"\n>>> Circle {circle_num}/3 - Executing trajectory "
                "(gripper stays closed, faster speed)..."
            )
            for i, vals in enumerate(circle_data, start=1):
                send_trajectory_point(ser, vals, i, len(circle_data), delay=CIRCLE_DELAY)
            print(f"\nCircle {circle_num}/3 complete!")

        print("\n" + "=" * 50)
        print("TRAJECTORY COMPLETE!")
        print("=" * 50)
    finally:
        ser.close()


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Stream precomputed 4-DOF trajectory CSVs to the Arduino controller."
    )
    parser.add_argument(
        "--port",
        default=DEFAULT_SERIAL_PORT,
        help=f"Serial port (default: {DEFAULT_SERIAL_PORT})",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUD_RATE,
        help=f"Serial baud rate (default: {DEFAULT_BAUD_RATE})",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Load and validate all 5 trajectory CSV files without opening a serial port.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        phases = load_all_trajectories()
    except (FileNotFoundError, ValueError) as exc:
        print(f"Error: {exc}")
        return 1

    if args.dry_run:
        run_dry_run(phases)
        return 0

    try:
        execute_serial_sequence(args.port, args.baud, phases)
    except Exception as exc:
        print(f"Error: Could not open or stream to port {args.port}: {exc}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
