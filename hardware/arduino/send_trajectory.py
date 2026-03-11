import serial
import time
import csv
import sys

# ================= CONFIGURATION =================
SERIAL_PORT = '/dev/cu.usbserial-110'      # WINDOWS: 'COM3', 'COM4', etc.
                          # LINUX/MAC: '/dev/ttyUSB0' or '/dev/ttyACM0'
BAUD_RATE = 115200        # Must match Arduino
CSV_FILE_1 = 'csv_files/trajectory_data_combined_1_square1.csv'  # First trajectory - Square 1
CSV_FILE_2 = 'csv_files/trajectory_data_combined_2_square2.csv'  # Second trajectory - Square 2
CSV_FILE_3 = 'csv_files/trajectory_data_combined_3_square2_rev.csv'  # Third trajectory - Square 2 Reverse
CSV_FILE_5 = 'csv_files/trajectory_data_combined_4_transition.csv'  # Fourth trajectory - Transition
CSV_FILE_4 = 'csv_files/trajectory_data_combined_5_circle.csv'  # Fifth trajectory - Circle
DELAY_BETWEEN_POINTS = 0.1 # Seconds (matches your old 100ms delay)
CIRCLE_DELAY = 0.05 # Faster delay for circle trajectory (50ms)

# Gripper Control
GRIPPER_OPEN_CMD = "GRIPPER_OPEN\n"
GRIPPER_CLOSE_CMD = "GRIPPER_CLOSE\n"
# =================================================

def send_gripper_command(ser, command_name, command):
    """Send a gripper command and wait for acknowledgment."""
    print(f"\n>>> {command_name}...")
    ser.write(command.encode('utf-8'))
    
    # Wait for READY acknowledgment
    while True:
        response = ser.readline().decode('utf-8').strip()
        if response:
            print(f"    Arduino: {response}")
        if "READY" in response:
            break

def send_trajectory_point(ser, vals, point_num, total, delay=None):
    """Send a single trajectory point and wait for acknowledgment."""
    if delay is None:
        delay = DELAY_BETWEEN_POINTS
    
    msg = f"{vals[0]},{vals[1]},{vals[2]},{vals[3]}\n"
    ser.write(msg.encode('utf-8'))
    
    # Log progress
    sys.stdout.write(f"\rSending Point {point_num}/{total}: {msg.strip()}   ")
    sys.stdout.flush()
    
    # Wait for acknowledgment
    while True:
        response = ser.readline().decode('utf-8').strip()
        if "READY" in response:
            break
    
    time.sleep(delay)

def main():
    try:
        # Open Serial Connection
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2) # Wait for Arduino to reset
        print(f"Connected to {SERIAL_PORT}")

        # Wait for handshake
        print("Waiting for Arduino boot...")
        while True:
            line = ser.readline().decode('utf-8').strip()
            if "ARDUINO_CONNECTED" in line:
                print("Arduino is online!")
                break

        # ========================================
        # LOAD ALL CSV FILES
        # ========================================
        print("\n" + "="*50)
        print("LOADING TRAJECTORY FILES")
        print("="*50)
        
        # Load Square 1
        with open(CSV_FILE_1, 'r') as f:
            square1_data = [row for row in csv.reader(f) if len(row) >= 4]
        print(f"Square 1: Loaded {len(square1_data)} points from {CSV_FILE_1}")
        
        # Load Square 2
        with open(CSV_FILE_2, 'r') as f:
            square2_data = [row for row in csv.reader(f) if len(row) >= 4]
        print(f"Square 2: Loaded {len(square2_data)} points from {CSV_FILE_2}")
        
        # Load Square 2 Reverse
        with open(CSV_FILE_3, 'r') as f:
            square2_reverse_data = [row for row in csv.reader(f) if len(row) >= 4]
        print(f"Square 2 Reverse: Loaded {len(square2_reverse_data)} points from {CSV_FILE_3}")
        
        # Load Transition Trajectory
        with open(CSV_FILE_5, 'r') as f:
            transition_data = [row for row in csv.reader(f) if len(row) >= 4]
        print(f"Transition: Loaded {len(transition_data)} points from {CSV_FILE_5}")
        
        # Load Circle
        with open(CSV_FILE_4, 'r') as f:
            circle_data = [row for row in csv.reader(f) if len(row) >= 4]
        print(f"Circle: Loaded {len(circle_data)} points from {CSV_FILE_4}")
        
        if len(square1_data) == 0 or len(square2_data) == 0 or len(square2_reverse_data) == 0 or len(transition_data) == 0 or len(circle_data) == 0:
            print("ERROR: One or more CSV files are empty!")
            return

        # ========================================
        # SQUARE 1: PICK AND MOVE
        # ========================================
        print("\n" + "="*50)
        print("EXECUTING SQUARE 1: PICK AND MOVE")
        print("="*50)
        
        # STEP 1: Open Gripper
        send_gripper_command(ser, "Opening Gripper", GRIPPER_OPEN_CMD)
        
        # STEP 2: Move to First Position (Pickup Position)
        print("\n>>> Moving to pickup position (first point)...")
        try:
            vals = [float(x) for x in square1_data[0][:4]]
            send_trajectory_point(ser, vals, 1, len(square1_data))
        except ValueError:
            print(f"ERROR: Invalid first position in Square 1")
            return
        
        # Wait 1 second for robot to stabilize at pickup position
        print("\nWaiting for robot to stabilize...")
        time.sleep(1.0)
        
        # STEP 3: Close Gripper (Grab Item)
        send_gripper_command(ser, "Closing Gripper to Grab Item", GRIPPER_CLOSE_CMD)
        
        # STEP 4: Execute Remaining Square 1 Trajectory (with gripper closed)
        print("\n>>> Executing Square 1 trajectory with item...")
        for i, row in enumerate(square1_data[1:], start=2):
            try:
                vals = [float(x) for x in row[:4]]
                send_trajectory_point(ser, vals, i, len(square1_data))
            except ValueError:
                print(f"\nSkipping invalid row in Square 1: {row}")
        
        # STEP 5: Open Gripper (Release Item)
        print("\n")
        send_gripper_command(ser, "Opening Gripper to Release Item", GRIPPER_OPEN_CMD)

        # ========================================
        # SQUARE 2: MOVE AND PLACE
        # ========================================
        print("\n" + "="*50)
        print("EXECUTING SQUARE 2: MOVE AND PLACE")
        print("="*50)
        
        # STEP 6: Execute ALL Square 2 Points (with gripper OPEN)
        print("\n>>> Executing Square 2 trajectory with gripper open...")
        for i, row in enumerate(square2_data, start=1):
            try:
                vals = [float(x) for x in row[:4]]
                send_trajectory_point(ser, vals, i, len(square2_data))
            except ValueError:
                print(f"\nSkipping invalid row in Square 2: {row}")
        
        # STEP 7: Wait 1 second at final position
        print("\nWaiting for robot to stabilize at placement position...")
        time.sleep(1.0)
        
        # STEP 8: Close Gripper (Place Item)
        print("\n")
        send_gripper_command(ser, "Closing Gripper to Place Item", GRIPPER_CLOSE_CMD)

        # ========================================
        # SQUARE 2 REVERSE: RETURN WITH GRIPPER CLOSED
        # ========================================
        print("\n" + "="*50)
        print("EXECUTING SQUARE 2 REVERSE: RETURN PATH")
        print("="*50)
        
        # STEP 9: Execute ALL Square 2 Reverse Points (with gripper CLOSED)
        print("\n>>> Executing Square 2 Reverse trajectory (gripper stays closed)...")
        for i, row in enumerate(square2_reverse_data, start=1):
            try:
                vals = [float(x) for x in row[:4]]
                send_trajectory_point(ser, vals, i, len(square2_reverse_data))
            except ValueError:
                print(f"\nSkipping invalid row in Square 2 Reverse: {row}")

        # ========================================
        # TRANSITION: PREPARE FOR CIRCLE
        # ========================================
        print("\n" + "="*50)
        print("EXECUTING TRANSITION TRAJECTORY")
        print("="*50)
        
        # STEP 10: Execute Transition Trajectory (gripper stays CLOSED)
        print("\n>>> Executing Transition trajectory (gripper stays closed)...")
        for i, row in enumerate(transition_data, start=1):
            try:
                vals = [float(x) for x in row[:4]]
                send_trajectory_point(ser, vals, i, len(transition_data))
            except ValueError:
                print(f"\nSkipping invalid row in Transition: {row}")

        # ========================================
        # CIRCLE: WITH GRIPPER CLOSED (3 TIMES)
        # ========================================
        print("\n" + "="*50)
        print("EXECUTING CIRCLE TRAJECTORY (3 REPETITIONS)")
        print("="*50)
        
        # STEP 11: Execute Circle 3 Times (with gripper CLOSED, faster speed)
        for circle_num in range(1, 4):  # Loop 3 times
            print(f"\n>>> Circle {circle_num}/3 - Executing trajectory (gripper stays closed, faster speed)...")
            for i, row in enumerate(circle_data, start=1):
                try:
                    vals = [float(x) for x in row[:4]]
                    send_trajectory_point(ser, vals, i, len(circle_data), delay=CIRCLE_DELAY)
                except ValueError:
                    print(f"\nSkipping invalid row in Circle: {row}")
            print(f"\nCircle {circle_num}/3 complete!")

        print("\n" + "="*50)
        print("TRAJECTORY COMPLETE!")
        print("="*50)
        ser.close()

    except serial.SerialException:
        print(f"Error: Could not open port {SERIAL_PORT}. Check connection.")
    except FileNotFoundError as e:
        print(f"Error: Could not find file - {e}")

if __name__ == "__main__":
    main()
