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

THRESH_X = 0.035  
THRESH_Y = 0.045  
BIAS_HORIZONTAL = 0.5   

TIME_LONG_MOVE = 5.0   
TIME_REVERSE_MOVE = 2.0 
TIME_SHORT_MOVE = 0.8  

BLINK_RATIO_LIMIT = 4.5
DOUBLE_BLINK_INTERVAL = 0.6
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
        
        self.last_x, self.last_y = 0.0, 0.0
        self.is_initialized = False 

    def update(self, raw_x, raw_y, is_blinking):
        if is_blinking: return self.last_x, self.last_y
            
        measurement = np.array([[raw_x], [raw_y]], dtype=np.float32)
        
        if not self.is_initialized:
            self.kf.statePost = np.array([[raw_x], [raw_y], [0.0], [0.0]], dtype=np.float32)
            self.last_x, self.last_y = raw_x, raw_y
            self.is_initialized = True
            return raw_x, raw_y

        self.kf.predict()
        self.kf.correct(measurement)
        prediction = self.kf.statePost
        self.last_x, self.last_y = prediction[0][0], prediction[1][0]
        return self.last_x, self.last_y

class GazeTracker:
    def __init__(self):
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            max_num_faces=1, refine_landmarks=True,  
            min_detection_confidence=0.8, min_tracking_confidence=0.8
        )

    def _euclidean_dist(self, p1, p2):
        return np.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

    def process_landmarks(self, landmarks, frame_w, frame_h):
        L_left_corner, L_right_corner = landmarks[33], landmarks[133] 
        L_top, L_bot, L_iris = landmarks[159], landmarks[145], landmarks[468]

        R_left_corner, R_right_corner = landmarks[362], landmarks[263] 
        R_top, R_bot, R_iris = landmarks[386], landmarks[374], landmarks[473]

        def get_euclidean_ratio(iris, p_left, p_right, p_top, p_bot):
            dist_left = self._euclidean_dist(iris, p_left)
            dist_right = self._euclidean_dist(iris, p_right)
            horiz_ratio = dist_left / (dist_left + dist_right + 1e-6)

            dist_top = self._euclidean_dist(iris, p_top)
            dist_bot = self._euclidean_dist(iris, p_bot)
            vert_ratio = dist_top / (dist_top + dist_bot + 1e-6)
            return horiz_ratio, vert_ratio

        lx_ratio, ly_ratio = get_euclidean_ratio(L_iris, L_left_corner, L_right_corner, L_top, L_bot)
        rx_ratio, ry_ratio = get_euclidean_ratio(R_iris, R_left_corner, R_right_corner, R_top, R_bot)

        avg_x, avg_y = (lx_ratio + rx_ratio) / 2.0, (ly_ratio + ry_ratio) / 2.0

        L_h, R_h = self._euclidean_dist(L_top, L_bot), self._euclidean_dist(R_top, R_bot)
        L_blink_ratio = self._euclidean_dist(L_left_corner, L_right_corner) / max(L_h, 0.001)
        R_blink_ratio = self._euclidean_dist(R_left_corner, R_right_corner) / max(R_h, 0.001)
        
        true_blink = min(L_blink_ratio, R_blink_ratio)

        l_px = (int(L_iris.x * frame_w), int(L_iris.y * frame_h))
        r_px = (int(R_iris.x * frame_w), int(R_iris.y * frame_h))

        return avg_x, avg_y, true_blink, l_px, r_px

class WheelchairDriver:
    def __init__(self):
        self.stabilizer = SignalStabilizer()
        self.calibrated = False
        self.center_x, self.center_y = 0.5, 0.5 
        self.current_state = "STOP"
        self.last_sent_cmd = ""
        self.pulse_active = False
        self.pulse_end_time = 0
        self.blink_active = False
        self.last_blink_time = 0
        self.front_dist, self.rear_dist = 999, 999
        self.command_queue = queue.Queue()
        self.running = True
        
        if ENABLE_BLUETOOTH: threading.Thread(target=self._bluetooth_worker, daemon=True).start()

    def _telemetry_handler(self, sender, data: bytearray):
        try:
            for p in data.decode("utf-8").split(","):
                if p.startswith("F:"): self.front_dist = int(p.split(":")[1])
                elif p.startswith("R:"): self.rear_dist = int(p.split(":")[1])
        except: pass 

    def _bluetooth_worker(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        async def async_task():
            while self.running: 
                print(f"[BLE] Scanning for {DEVICE_NAME}...")
                try:
                    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=5.0)
                    if not device: continue
                    print(f"[BLE] ✅ CONNECTED!")
                    async with BleakClient(device) as client:
                        await client.start_notify(CHARACTERISTIC_UUID_TX, self._telemetry_handler)
                        last_msg = ""
                        while self.running and client.is_connected:
                            try:
                                cmd = self.command_queue.get(timeout=0.1)
                                char_code = {"FORWARD": b'F', "REVERSE": b'B', "LEFT": b'L', "RIGHT": b'R', "STOP": b'S', "IDLE": b'S'}.get(cmd, b'S')
                                if cmd == "STOP":
                                    await client.write_gatt_char(CHARACTERISTIC_UUID_RX, b'S', response=False)
                                    last_msg = cmd
                                elif cmd != last_msg:
                                    print(f"[BLE] Sending: {cmd}")
                                    await client.write_gatt_char(CHARACTERISTIC_UUID_RX, char_code, response=False)
                                    last_msg = cmd
                            except queue.Empty: continue
                            except: break 
                        print("[BLE] Disconnected. Reconnecting...")
                except: await asyncio.sleep(2.0)
        loop.run_until_complete(async_task())
        loop.close()

    def send_command(self, cmd):
        if ENABLE_BLUETOOTH: self.command_queue.put(cmd)

    def update_logic(self, raw_x, raw_y, blink_val):
        now = time.time()
        
        if blink_val > BLINK_RATIO_LIMIT:
            if not self.blink_active:
                self.blink_active = True
                if (now - self.last_blink_time) < DOUBLE_BLINK_INTERVAL:
                    self.current_state = "IDLE" if self.current_state == "STOP" else "STOP"
                    self.pulse_active = False
                    self.last_blink_time = 0 
                    return self.center_x, self.center_y, self.current_state, blink_val
                self.last_blink_time = now
        else:
            self.blink_active = False

        smooth_x, smooth_y = self.stabilizer.update(raw_x, raw_y, self.blink_active)
        if self.current_state == "STOP": return smooth_x, smooth_y, "STOP", blink_val

        if self.pulse_active:
            if now < self.pulse_end_time: return smooth_x, smooth_y, self.current_state, blink_val
            else:
                self.pulse_active = False
                self.current_state = "IDLE"
                return smooth_x, smooth_y, "IDLE", blink_val

        if self.calibrated:
            dx, dy = smooth_x - self.center_x, smooth_y - self.center_y
            mag_x, mag_y = abs(dx), abs(dy)
            new_cmd = "IDLE"

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

        return smooth_x, smooth_y, self.current_state, blink_val

    def calibrate(self, x, y):
        self.center_x, self.center_y = x, y
        self.calibrated = True

def run_system():
    driver = WheelchairDriver()
    tracker = GazeTracker()
    
    cap = cv2.VideoCapture(1) 
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    
    print("--- SYSTEM READY: Press 'C' to Calibrate, 'ESC' to Quit ---")
    
    while True:
        ret, frame = cap.read()
        if not ret: break
        
        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = tracker.face_mesh.process(rgb)

        if results.multi_face_landmarks:
            landmarks = results.multi_face_landmarks[0].landmark
            
            rx, ry, true_blink, left_px, right_px = tracker.process_landmarks(landmarks, w, h)
            sx, sy, state, current_blink_ratio = driver.update_logic(rx, ry, true_blink)
            
            if state != driver.last_sent_cmd:
                driver.send_command(state)
                driver.last_sent_cmd = state
            
            if cv2.waitKey(1) in [ord('c'), ord('C')]: driver.calibrate(sx, sy)

            # UI Graphics
            color = (0, 0, 255) if state == "STOP" else (0, 255, 255) if state == "IDLE" else (0, 255, 0)
            
            if not driver.blink_active:
                cv2.drawMarker(frame, left_px, (0, 255, 0), cv2.MARKER_CROSS, 15, 2)
                cv2.drawMarker(frame, right_px, (0, 255, 0), cv2.MARKER_CROSS, 15, 2)

            cx, cy = w // 2, h // 2

            if driver.calibrated:
                cv2.drawMarker(frame, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)
                gx = int(cx + (sx - driver.center_x) * w * 1.5)
                gy = int(cy + (sy - driver.center_y) * h * 1.5)
                cv2.arrowedLine(frame, (cx, cy), (gx, gy), color, 3, tipLength=0.3)
                
                box_w, box_h = int(THRESH_X * w * 1.5), int(THRESH_Y * h * 1.5)
                cv2.rectangle(frame, (cx - box_w, cy - box_h), (cx + box_w, cy + box_h), (255, 255, 255), 1)
            else:
                cv2.putText(frame, "LOOK CENTER & PRESS 'C'", (w//4, h//2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            
            cv2.putText(frame, f"CMD: {state}", (20, 50), cv2.FONT_HERSHEY_DUPLEX, 1.0, color, 2)
            
            blink_color = (0,0,255) if driver.blink_active else (255,255,255)
            cv2.putText(frame, f"Blink Ratio: {current_blink_ratio:.1f}", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, blink_color, 2)
            
            # Telemetry Display Overlay
            f_text = f"FRONT: {driver.front_dist}cm" if driver.front_dist != 999 else "FRONT: CLEAR"
            r_text = f"REAR: {driver.rear_dist}cm" if driver.rear_dist != 999 else "REAR: CLEAR"
            cv2.putText(frame, f_text, (w - 300, 50), cv2.FONT_HERSHEY_DUPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, r_text, (w - 300, 90), cv2.FONT_HERSHEY_DUPLEX, 0.7, (255, 255, 255), 2)

        # Output the video at 50% scale for screen space optimization
        display_frame = cv2.resize(frame, (960, 540))
        cv2.imshow("IrisDrive HUD Unit", display_frame)
        
        if cv2.waitKey(1) == 27: break

    driver.running = False
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    run_system()