# 🤖 Tea-Making Robot Arm - User Guide

## 📋 Overview

This project controls a **4-DOF (4 Degrees of Freedom) robotic arm** designed to perform automated tea-making tasks. The robot features **4 joints** for precise movement and a **gripper** (not counted as a DOF) for object manipulation.

### Robot Specifications
- **Degrees of Freedom:** 4 (Joints 1-4)
- **End Effector:** 1 Gripper (Joint 5)
- **Control:** Serial communication via Arduino
- **Function:** Automated tea preparation and serving

---

## 🎯 System Components

### Hardware
1. **Arduino Board** (with servo connections)
2. **5 Servo Motors:**
   - Joint 1-4: Robot arm movement (4-DOF)
   - Joint 5: Gripper for grasping objects
3. **USB Cable** for Arduino-to-Computer connection

### Software Files
1. **`receiver.ino`** - Arduino firmware (runs on Arduino)
2. **`send_trajectory.py`** - Python trajectory controller (runs on computer)
3. **CSV Trajectory Files** (in `trajectories/` folder):
   - `trajectory_data_combined_1_square1.csv`
   - `trajectory_data_combined_2_square2.csv`
   - `trajectory_data_combined_3_square2_rev.csv`
   - `trajectory_data_combined_4_transition.csv`
   - `trajectory_data_combined_5_circle.csv`

---

## 🚀 Setup Instructions

### Step 1: Upload Arduino Firmware

1. **Open Arduino IDE**
   - Download from [arduino.cc](https://www.arduino.cc/en/software) if not installed

2. **Open the receiver sketch**
   - File → Open → Navigate to `receiver/receiver.ino`

3. **Select your Arduino board**
   - Tools → Board → Select your Arduino model (e.g., Arduino Uno)

4. **Select the correct port**
   - Tools → Port → Select the port with your Arduino
   - Windows: `COM3`, `COM4`, etc.
   - Mac/Linux: `/dev/cu.usbserial-110` or similar

5. **Upload the code**
   - Click the **Upload** button (→) or press `Ctrl+U` (Windows) / `Cmd+U` (Mac)
   - Wait for "Done uploading" message

6. **Verify connection**
   - Open Serial Monitor (Tools → Serial Monitor)
   - Set baud rate to **115200**
   - You should see: `ARDUINO_CONNECTED`

---

### Step 2: Run the Python Trajectory Controller

#### Prerequisites
Make sure you have Python 3 installed with the required library:
```bash
pip install pyserial
```

#### Configuration
1. **Open `send_trajectory.py`** in a text editor
2. **Update the serial port** (lines 7-9) to match your Arduino port:
   ```python
   SERIAL_PORT = '/dev/cu.usbserial-110'  # Mac/Linux
   # or
   SERIAL_PORT = 'COM3'  # Windows
   ```

#### Running the Program
1. **Open a terminal/command prompt** in the `` directory

2. **Run the script:**
   ```bash
   python send_trajectory.py
   ```

3. **Expected sequence:**
   - Script connects to Arduino
   - Loads 5 trajectory CSV files
   - Executes the tea-making sequence automatically

---

## 🎬 Tea-Making Sequence

The robot performs the following automated sequence:

### **Phase 1: Square 1 - Pick Tea Bag** (104 points)
1. 🟢 **Opens gripper**
2. 📍 **Moves to pickup position**
3. 🔴 **Closes gripper** (grabs tea bag)
4. 🚶 **Moves tea bag** along square trajectory
5. 🟢 **Opens gripper** (releases tea bag into cup)

### **Phase 2: Square 2 - Reposition** (104 points)
1. 🚶 **Moves to new location** (gripper open)
2. 🔴 **Closes gripper** (prepares for next task)

### **Phase 3: Square 2 Reverse - Return** (104 points)
1. 🚶 **Returns along reverse path** (gripper closed)

### **Phase 4: Transition - Prepare for Stirring** (42 points)
1. 🚶 **Transitions to stirring position** (gripper closed)

### **Phase 5: Circle - Stir Tea** (102 points × 3 repetitions)
1. 🔁 **Performs circular stirring motion** 3 times
2. ⚡ **Faster speed** for efficient stirring

**Total Execution Time:** ~49 seconds

---

## ⚙️ Gripper Configuration

### Safety Settings
The gripper is configured to prevent motor overload:

```cpp
GRIPPER_OPEN = 90.0°     → Servo: 180° (fully open)
GRIPPER_CLOSED = 30.0°   → Servo: 120° (safe grip)
GRIPPER_MIN_SAFE = 30.0° → Safety limit
```

### Adjusting Grip Strength
If objects are slipping or motor is struggling, edit `receiver.ino`:

**For tighter grip:**
```cpp
const float GRIPPER_CLOSED = 15.0;  // Tighter (Servo: 105°)
```

**For gentler grip:**
```cpp
const float GRIPPER_CLOSED = 40.0;  // Gentler (Servo: 130°)
```

⚠️ **Warning:** Never set below 10° to avoid motor damage!

---

## � Trajectory File Format

Each CSV file contains joint angles (in degrees):
```
J1, J2, J3, J4
33.65, -4.69, 57.08, -89.94
...
```

**Column Layout:**
- **Column 1:** Joint 1 angle (degrees)
- **Column 2:** Joint 2 angle (degrees)
- **Column 3:** Joint 3 angle (degrees)
- **Column 4:** Joint 4 angle (degrees)

The gripper is controlled separately via commands, not trajectory files.

---

## � Troubleshooting

### Arduino Not Found
- Check USB connection
- Verify correct port in Arduino IDE and `send_trajectory.py`
- Try unplugging and reconnecting Arduino

### Python Script Errors
**"Could not open port":**
- Close Arduino Serial Monitor before running script
- Update `SERIAL_PORT` in `send_trajectory.py`

**"Module not found (serial)":**
```bash
pip install pyserial
```

### Gripper Issues
**Not gripping tightly enough:**
- Reduce `GRIPPER_CLOSED` angle (e.g., from 30° to 15°)

**Motor struggling/buzzing:**
- Increase `GRIPPER_CLOSED` angle (e.g., from 15° to 40°)

### Trajectory Issues
**Robot jerky movement:**
- Increase `DELAY_BETWEEN_POINTS` in `send_trajectory.py` (line 15)

**CSV file not found:**
- Ensure all CSV files are in `trajectories/` folder
- Check file names match exactly

---

## 🎛️ Customization

### Modify Speed
Edit `send_trajectory.py`:
```python
DELAY_BETWEEN_POINTS = 0.1  # Slower: increase (e.g., 0.2)
CIRCLE_DELAY = 0.05         # Faster stirring: decrease (e.g., 0.03)
```

### Change Trajectory
Replace CSV files in `trajectories/` folder with your own trajectory data. Maintain the same format (4 columns for Joint 1-4).

### Add New Phases
Edit `send_trajectory.py` to add more CSV files and execution steps following the existing pattern.

---

## 📝 Technical Details

### Communication Protocol
- **Baud Rate:** 115200
- **Command Format:**
  - Joint angles: `J1,J2,J3,J4\n`
  - Gripper open: `GRIPPER_OPEN\n`
  - Gripper close: `GRIPPER_CLOSE\n`
- **Acknowledgment:** Arduino sends `READY` after each command

### Joint Calibration
Each joint has specific calibration offsets (see `receiver.ino`):
- **Joint 1:** Direct mapping + 90° offset
- **Joint 2:** Inverted + 110° offset
- **Joint 3:** Inverted + 90° offset
- **Joint 4:** Inverted + 110° offset
- **Gripper:** Direct mapping + 90° offset

---

## 📞 Support

For issues or questions:
1. Check the Troubleshooting section
2. Verify all connections and settings
3. Review serial monitor output for error messages

---

## 🏆 Project Summary

This 4-DOF robotic tea maker demonstrates:
- ✅ Automated trajectory execution
- ✅ Precise gripper control
- ✅ Multi-phase task sequencing
- ✅ Safe motor operation
- ✅ Serial communication between Arduino and Python

**Enjoy your automated tea! ☕🤖**

