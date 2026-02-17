# 🚀 Engineering Change Log: Gaze Tracking System

## 1. System Architecture
* **OLD (Script-based):** A single `while True` loop mixing UI, Math, and Logic. Hard to debug.
* **NEW (Class-based):** Separated into 3 modules:
    * `GazeTracker`: Handles Camera & MediaPipe math.
    * `SignalStabilizer`: Handles Physics smoothing.
    * `WheelchairDriver`: Handles Bluetooth & Decision Logic.

## 2. Signal Stability (The "Jitter" Fix)
* **OLD:** Used raw coordinates from the camera.
    * *Result:* The green arrow flickered rapidly even when the head was still.
* **NEW:** Implemented a **Kalman Filter** (`cv2.KalmanFilter`).
    * *Result:* Calculates velocity and position to predict movement. The arrow now "glides" smoothly, eliminating micro-tremors.

## 3. Bluetooth Reliability (The "Ghost" Fix)
* **OLD:** No threading logic.
    * *Result:* Running Bluetooth inside the Camera loop caused Windows to crash (`Assert MTA` error) or froze the video feed.
* **NEW:** **Background Daemon Thread**.
    * *Result:* Bluetooth runs in a parallel universe. It uses a `Queue` system to talk to the camera. Even if Bluetooth lags, the camera never freezes.
    * *Feature:* **Auto-Reconnect**. If the wheelchair disconnects, the script waits 2 seconds and automatically reconnects without restarting the app.

## 4. Hardware Safety (The "Spam" Fix)
* **OLD:** Sent commands continuously (e.g., `F, F, F, F...` 30 times/sec).
    * *Result:* Flooded the ESP32 buffer, causing lag and "runaway" robot issues.
* **NEW:** **State-Change Logic**.
    * *Result:* Only sends a command **IF** it changes (e.g., `Stop -> Forward`).
    * *Feature:* **Hysteresis (Cooldown)**. After turning, the system locks for 0.8s to prevent accidental rapid direction flipping.