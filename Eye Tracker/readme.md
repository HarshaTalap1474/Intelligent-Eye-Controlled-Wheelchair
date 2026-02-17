# GazeLink: Contactless Eye-Tracking Controller

**GazeLink** is a lightweight, cross-platform machine learning system that converts eye movements into discrete hardware control signals. Designed for low-compute environments (Windows/Linux), it leverages Google’s MediaPipe Iris Mesh to detect gaze direction in real time without requiring specialized calibration equipment.

It is specifically engineered for hardware control applications such as motorized wheelchairs and robotic arms. The system incorporates a **State Latching Mechanism** to prevent jitter and eliminate accidental command triggers.

---

## ⚡ Key Features

- **Real-Time Iris Tracking**  
  Utilizes a 468-point face mesh model for sub-millimeter precision.

- **State Latching Engine**  
  Locks a command (e.g., `FORWARD`) until explicitly reset, preventing motor stutter.

- **Adaptive Calibration**  
  One-click relative calibration adjusts instantly to user position, camera angle, and lighting.

- **Horizontal Bias Logic**  
  Filters natural eye droop to prevent unintended `REVERSE` triggers during left/right turns.

- **Double-Blink Switch**  
  Integrated safety kill-switch for Start/Stop control.

- **Lightweight Execution**  
  CPU-compatible (runs on Intel i3 and Raspberry Pi 4).

---

## 🛠️ System Architecture

GazeLink operates as a finite state machine with three primary states:

### States

- **STOP (Safe Mode)**  
  System ignores all gaze inputs.

- **IDLE (Ready Mode)**  
  System awaits a deliberate gaze gesture.

- **LOCKED (Active Mode)**  
  System latches onto a direction (`LEFT`, `RIGHT`, `FORWARD`, `REVERSE`) and maintains output.

### Transition Logic

- `STOP → IDLE` : Double Blink  
- `IDLE → LOCKED` : Gaze direction exceeds sensitivity threshold  
- `LOCKED → STOP` : Double Blink (Emergency Reset)

---

## 📦 Installation

### Prerequisites

- **OS**: Windows 10/11 or Linux (Manjaro/Ubuntu)
- **Python**: Version 3.10 (strictly required due to MediaPipe compatibility)
- **Hardware**: Standard webcam (720p recommended)

---

### 1. Clone the Repository

```bash
git clone https://github.com/yourusername/gazelink.git
cd gazelink
```

---

### 2. Set Up Virtual Environment

#### Option A: Conda (Recommended for Linux/Manjaro)

```bash
conda create -n gazelink python=3.10 -y
conda activate gazelink
```

#### Option B: Standard Python (Windows)

```bash
python -m venv venv
.\venv\Scripts\activate
```

---

### 3. Install Dependencies

```bash
pip install -r requirements.txt
```

#### `requirements.txt`

```plaintext
opencv-python>=4.8.0
mediapipe==0.10.9
numpy>=1.21.0
```

---

## 🚀 Usage Guide

### 1. Run the Controller

```bash
python main.py
```

---

### 2. Calibration Step

1. The system launches in **Gray Mode**.
2. Sit comfortably in front of the camera.
3. Look at the center of the screen.
4. Press **`C`** on the keyboard.

The system will calibrate to your resting facial position.

---

### 3. Control Flow

#### Unlock
- Blink twice quickly  
- Interface turns **Yellow (IDLE)**

#### Drive
- Look **Left/Right** → Turn
- Look **Up** → Forward
- Look **Down** → Reverse
- System turns **Green (LOCKED)** and maintains command

#### Stop
- Blink twice quickly  
- System resets to **Red (STOP)**

---

## ⚙️ Configuration & Tuning

Open `main.py` to modify sensitivity parameters.

### Sensitivity Parameters

- `SENSITIVITY_X`
- `SENSITIVITY_Y`

These determine how far the user must look to trigger a command.

- **Lower Value (0.02)** → Highly sensitive (suitable for limited mobility)
- **Higher Value (0.08)** → Requires deliberate movement (reduces false triggers)

```python
# Standard Setup
SENSITIVITY_X = 0.035
SENSITIVITY_Y = 0.06  # Higher due to noisier downward gaze detection
```

---

### Horizontal Bias

`HORIZONTAL_BIAS` controls directional priority.

- `0.5` (Default)  
  Prioritizes Left/Right unless strong vertical movement is detected.

- `1.0`  
  Equal weighting for vertical and horizontal directions.

---

## 🔌 Hardware Integration

To integrate with Arduino, ESP32, or robotic platforms, modify the signal output section in `main.py`:

```python
# Locate this block in main.py

if current_state != last_printed_state:
    print(f"DEVICE_SIGNAL: {current_state}")
    
    # Insert serial communication code here:
    # serial_port.write(current_state.encode())
    
    last_printed_state = current_state
```

---

## 🔧 Troubleshooting

| Issue | Possible Cause | Solution |
|-------|----------------|----------|
| Crash on start | Incorrect Python version | Verify Python 3.10 using `python --version` |
| Stuck on "FORWARD" | Screen angle misalignment | Adjust laptop screen or press `C` to recalibrate |
| "Module Not Found" | Virtual environment inactive | Run `conda activate gazelink` or activate venv |
| Cannot trigger "REVERSE" | Sensitivity too high | Reduce `SENSITIVITY_Y` to 0.05 |

---

## 📜 License

Distributed under the **MIT License**. See `LICENSE` file for details.

---

## 👤 Maintainer

Developed for Assistive Technology Applications  
Maintained by **Harsh Chinchakar**
