import cv2
import mediapipe as mp
import numpy as np
import time
import threading
import queue
import asyncio
from bleak import BleakScanner, BleakClient

# ==============================================================================
#                             CONFIG & TUNING
# ==============================================================================
DEVICE_NAME = "Wheelchair_BLE"
CHARACTERISTIC_UUID_RX = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
CHARACTERISTIC_UUID_TX = "1c28c688-29ce-44d5-ab46-5991444903bc"
ENABLE_BLUETOOTH = True 

# JOYSTICK DEADZONE THRESHOLDS (Tune these if it's too sensitive)
# Higher number = you have to turn your head further to move
THRESH_X = 0.015
THRESH_Y = 0.025
BIAS_HORIZONTAL = 0.6   

# Pulse Timing (Matches ESP32 Hardware Logic)
TIME_LONG_MOVE = 5.0   
TIME_REVERSE_MOVE = 2.0 
TIME_SHORT_MOVE = 0.8  

# Emergency Stop Settings
BLINK_RATIO_LIMIT = 5.0
DOUBLE_BLINK_INTERVAL = 0.6

# Kalman Filter Tuning (Smoothes out the nose tracking)
PROCESS_NOISE = 1e-4    
MEASURE_NOISE = 0.5    

# ==============================================================================
#                            CORE CLASSES
# ==============================================================================

class SignalStabilizer:
    def __init__(self):
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        self.kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * PROCESS_NOISE
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * MEASURE_NOISE

    def update(self, raw_x, raw_y):
        self.kf.predict()
        measurement = np.array([[np.float32(raw_x)], [np.float32(raw_y)]])
        self.kf.correct(measurement)
        prediction = self.kf.statePost
        return prediction[0][0], prediction[1][0]

class FaceJoystick:
    def __init__(self):
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=True,
            min_detection_confidence=0.8,
            min_tracking_confidence=0.8
        )

    def _euclidean_dist(self, p1, p2):
        return np.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

    def process_landmarks(self, landmarks):
        # 1. THE JOYSTICK: Track the tip of the nose (Landmark 4)
        nose = landmarks[4]
        nose_x, nose_y = nose.x, nose.y

        # 2. THE KILL SWITCH: Check for blinks (Averaging both eyes)
        # Left Eye
        L_top, L_bot = landmarks[159], landmarks[145]
        L_in, L_out = landmarks[33], landmarks[133]
        L_h = self._euclidean_dist(L_top, L_bot)
        L_w = self._euclidean_dist(L_in, L_out)
        L_blink = L_w / L_h if L_h > 0 else 0

        # Right Eye
        R_top, R_bot = landmarks[386], landmarks[374]
        R_in, R_out = landmarks[362], landmarks[263]
        R_h = self._euclidean_dist(R_top, R_bot)
        R_w = self._euclidean_dist(R_in, R_out)
        R_blink = R_w / R_h if R_h > 0 else 0

        avg_blink = (L_blink + R_blink) / 2.0
        
        return nose_x, nose_y, avg_blink

class WheelchairDriver:
    def __init__(self):
        self.stabilizer = SignalStabilizer()
        self.calibrated = False
        self.center_x = 0.0
        self.center_y = 0.0
        
        self.current_state = "STOP"
        self.last_sent_cmd = ""
        
        self.pulse_active = False
        self.pulse_end_time = 0
        self.blink_active = False
        self.last_blink_time = 0
        
        self.front_dist = 999
        self.rear_dist = 999
        
        self.command_queue = queue.Queue()
        self.running = True
        
        if ENABLE_BLUETOOTH:
            self.bt_thread = threading.Thread(target=self._bluetooth_worker, daemon=True)
            self.bt_thread.start()

    def _telemetry_handler(self, sender, data: bytearray):
        try:
            msg = data.decode("utf-8")
            parts = msg.split(",")
            for p in parts:
                if p.startswith("F:"): self.front_dist = int(p.split(":")[1])
                elif p.startswith("R:"): self.rear_dist = int(p.split(":")[1])
        except Exception:
            pass 

    def _bluetooth_worker(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        async def async_task():
            while self.running: 
                print(f"[BLE] Scanning for {DEVICE_NAME}...")
                try:
                    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=5.0)
                    if not device:
                        await asyncio.sleep(2.0)
                        continue

                    print(f"[BLE] ✅ Connecting...")
                    async with BleakClient(device) as client:
                        print(f"[BLE] ✅ CONNECTED!")
                        await client.start_notify(CHARACTERISTIC_UUID_TX, self._telemetry_handler)
                        
                        last_msg = ""
                        while self.running and client.is_connected:
                            try:
                                cmd = self.command_queue.get(timeout=0.1)
                                protocol_map = {"FORWARD": b'F', "REVERSE": b'B', "LEFT": b'L', "RIGHT": b'R', "STOP": b'S', "IDLE": b'S'}
                                char_code = protocol_map.get(cmd, b'S')
                                
                                if cmd == "STOP":
                                    await client.write_gatt_char(CHARACTERISTIC_UUID_RX, b'S', response=False)
                                    last_msg = cmd
                                elif cmd != last_msg:
                                    print(f"[BLE] Sending: {cmd}")
                                    await client.write_gatt_char(CHARACTERISTIC_UUID_RX, char_code, response=False)
                                    last_msg = cmd
                            except queue.Empty: continue
                            except Exception: break 
                        print("[BLE] Disconnected. Reconnecting...")
                except Exception:
                    await asyncio.sleep(2.0)

        loop.run_until_complete(async_task())
        loop.close()

    def send_command(self, cmd):
        if ENABLE_BLUETOOTH: self.command_queue.put(cmd)

    def update_logic(self, raw_x, raw_y, blink_val):
        now = time.time()
        smooth_x, smooth_y = self.stabilizer.update(raw_x, raw_y)

        # 1. EMERGENCY STOP (Double Blink)
        if blink_val > BLINK_RATIO_LIMIT:
            if not self.blink_active:
                self.blink_active = True
                if (now - self.last_blink_time) < DOUBLE_BLINK_INTERVAL:
                    self.current_state = "IDLE" if self.current_state == "STOP" else "STOP"
                    self.pulse_active = False
                    self.last_blink_time = 0 
                    return smooth_x, smooth_y, self.current_state
                self.last_blink_time = now
        else:
            self.blink_active = False

        if self.current_state == "STOP": return smooth_x, smooth_y, "STOP"

        # 2. PULSE TIMER
        if self.pulse_active:
            if now < self.pulse_end_time: return smooth_x, smooth_y, self.current_state
            else:
                self.pulse_active = False
                self.current_state = "IDLE"
                return smooth_x, smooth_y, "IDLE"

        # 3. JOYSTICK KINEMATICS
        if self.calibrated:
            dx = smooth_x - self.center_x
            dy = smooth_y - self.center_y
            mag_x, mag_y = abs(dx), abs(dy)
            new_cmd = "IDLE"

            # Check if nose has left the "Deadzone"
            if mag_x > THRESH_X and mag_x > (mag_y * BIAS_HORIZONTAL):
                new_cmd = "LEFT" if dx < 0 else "RIGHT"
            elif mag_y > THRESH_Y:
                new_cmd = "FORWARD" if dy < 0 else "REVERSE"
            
            if new_cmd != "IDLE":
                self.current_state = new_cmd
                self.pulse_active = True
                if new_cmd == "FORWARD": self.pulse_end_time = now + TIME_LONG_MOVE
                elif new_cmd == "REVERSE": self.pulse_end_time = now + TIME_REVERSE_MOVE
                else: self.pulse_end_time = now + TIME_SHORT_MOVE

        return smooth_x, smooth_y, self.current_state

    def calibrate(self, x, y):
        self.center_x = x
        self.center_y = y
        self.calibrated = True

def get_dist_color(dist):
    if dist > 60: return (0, 255, 0)      
    if dist > 30: return (0, 255, 255)    
    return (0, 0, 255)                    

def run_system():
    driver = WheelchairDriver()
    joystick = FaceJoystick()
    
    # EXTERNAL CAMERA SETUP WITH HIGH-RES FIX
    cap = cv2.VideoCapture(1) 
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 720)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    print("--- SYSTEM READY: Press 'C' to Calibrate, 'ESC' to Quit ---")
    
    # We don't need digital zoom anymore because head tracking doesn't need to be macroscopic!
    
    while True:
        ret, frame = cap.read()
        if not ret: break
        
        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape
        
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = joystick.face_mesh.process(rgb)

        if results.multi_face_landmarks:
            landmarks = results.multi_face_landmarks[0].landmark
            
            nx, ny, blink = joystick.process_landmarks(landmarks)
            sx, sy, state = driver.update_logic(nx, ny, blink)
            
            if state != driver.last_sent_cmd:
                driver.send_command(state)
                driver.last_sent_cmd = state
            
            if cv2.waitKey(1) in [ord('c'), ord('C')]:
                driver.calibrate(sx, sy)

            # --- VISUAL HUD & VIRTUAL JOYSTICK ---
            color = (0, 0, 255) 
            if state == "IDLE": color = (0, 255, 255)
            elif state != "STOP": color = (0, 255, 0)
            
            if driver.calibrated:
                cx, cy = int(driver.center_x * w), int(driver.center_y * h)
                current_x, current_y = int(sx * w), int(sy * h)
                
                # Draw the Deadzone Box
                box_w, box_h = int(THRESH_X * w), int(THRESH_Y * h)
                cv2.rectangle(frame, (cx - box_w, cy - box_h), (cx + box_w, cy + box_h), (255, 255, 255), 1)
                
                # Draw the Joystick Stick (Center to Nose)
                cv2.line(frame, (cx, cy), (current_x, current_y), color, 2)
                cv2.circle(frame, (current_x, current_y), 8, color, -1)
                
                if driver.pulse_active:
                    remaining = int(driver.pulse_end_time - time.time())
                    cv2.putText(frame, f"DRIVING... {remaining}s", (current_x + 15, current_y - 15), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(frame, "LOOK CENTER & PRESS 'C'", (w//4, h//2), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            
            cv2.putText(frame, f"CMD: {state}", (20, 50), cv2.FONT_HERSHEY_DUPLEX, 1.0, color, 2)

            # TELEMETRY DISPLAY
            f_color = get_dist_color(driver.front_dist)
            r_color = get_dist_color(driver.rear_dist)
            f_text = f"FRONT: {driver.front_dist}cm" if driver.front_dist != 999 else "FRONT: CLEAR"
            r_text = f"REAR: {driver.rear_dist}cm" if driver.rear_dist != 999 else "REAR: CLEAR"
            cv2.putText(frame, f_text, (w - 350, 50), cv2.FONT_HERSHEY_DUPLEX, 0.8, f_color, 2)
            cv2.putText(frame, r_text, (w - 350, 90), cv2.FONT_HERSHEY_DUPLEX, 0.8, r_color, 2)

            if driver.front_dist < 30 or driver.rear_dist < 30:
                cv2.putText(frame, "OBSTACLE WARNING", (w//2 - 220, h//2 + 120), cv2.FONT_HERSHEY_DUPLEX, 1.0, (0, 0, 255), 3)
        
        cv2.imshow("IrisDrive HUD Unit", frame)
        if cv2.waitKey(1) == 27: break

    driver.running = False
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    run_system()