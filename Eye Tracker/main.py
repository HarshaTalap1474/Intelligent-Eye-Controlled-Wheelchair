import cv2
import mediapipe as mp
import numpy as np
import time

# --- SENSITIVITY CONFIGURATION ---
# Lower = More Sensitive. Higher = Harder to trigger.
SENSITIVITY_X = 0.035  # Very sensitive for Left/Right
SENSITIVITY_Y = 0.06   # Less sensitive for Up/Down (Harder to trigger)

# "Cone" Logic: How strict is the vertical check?
# 0.5 means: "Unless Vertical movement is 2x stronger than Horizontal, assume Horizontal."
# This creates a wide "Left/Right" zone and a narrow "Up/Down" strip.
HORIZONTAL_BIAS = 0.5 

BLINK_THRESHOLD = 5.0
DOUBLE_BLINK_SPEED = 0.6 

# --- INIT MEDIAPIPE ---
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(
    max_num_faces=1,
    refine_landmarks=True, 
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7
)

# --- UTILS ---
def get_euclidean_distance(p1, p2):
    return np.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

def get_iris_position(landmarks):
    # RIGHT EYE INDICES
    p_inner = landmarks[33] 
    p_outer = landmarks[133]
    p_iris = landmarks[468]
    p_top = landmarks[159]
    p_bottom = landmarks[145]
    
    # 1. Horizontal Ratio
    eye_width = get_euclidean_distance(p_inner, p_outer)
    dist_to_inner = get_euclidean_distance(p_iris, p_inner)
    x_ratio = dist_to_inner / (eye_width + 1e-6)
    
    # 2. Vertical Ratio
    eye_height = get_euclidean_distance(p_top, p_bottom)
    dist_to_top = get_euclidean_distance(p_iris, p_top)
    y_ratio = dist_to_top / (eye_height + 1e-6)
    
    return x_ratio, y_ratio

def get_blink_ratio(landmarks):
    top = landmarks[159]
    bottom = landmarks[145]
    v_dist = get_euclidean_distance(top, bottom)
    inner = landmarks[33]
    outer = landmarks[133]
    h_dist = get_euclidean_distance(inner, outer)
    return h_dist / (v_dist + 1e-6)

# --- MAIN LOOP ---
cap = cv2.VideoCapture(0)

# State Variables
calibrated = False
center_x, center_y = 0.0, 0.0
current_state = "STOP"
last_printed_state = ""
last_blink_time = 0
is_blinking = False

print("--- HORIZONTAL-BIASED CONTROLLER ---")
print("1. Look at CENTER and press 'C' to Calibrate.")

while True:
    ret, frame = cap.read()
    if not ret: break
    
    frame = cv2.flip(frame, 1)
    h, w, c = frame.shape
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = face_mesh.process(rgb_frame)
    
    if results.multi_face_landmarks:
        landmarks = results.multi_face_landmarks[0].landmark
        
        # 1. Get Data
        raw_x, raw_y = get_iris_position(landmarks)
        blink_ratio = get_blink_ratio(landmarks)
        
        # --- CALIBRATION ---
        key = cv2.waitKey(1)
        if key == ord('c') or key == ord('C'):
            center_x, center_y = raw_x, raw_y
            calibrated = True
            current_state = "IDLE"
            print(f"--- RE-CALIBRATED: {center_x:.2f}, {center_y:.2f} ---")

        if not calibrated:
            cv2.rectangle(frame, (0,0), (w, h), (20, 20, 20), -1)
            cv2.putText(frame, "LOOK CENTER & PRESS 'C'", (50, h//2), 
                        cv2.FONT_HERSHEY_DUPLEX, 1, (0, 255, 255), 2)
            cv2.imshow("Eye Controller", frame)
            continue

        # --- BLINK LOGIC ---
        if blink_ratio > BLINK_THRESHOLD:
            if not is_blinking:
                is_blinking = True
                curr_time = time.time()
                if (curr_time - last_blink_time) < DOUBLE_BLINK_SPEED:
                    if current_state == "STOP":
                        current_state = "IDLE"
                    else:
                        current_state = "STOP"
                last_blink_time = curr_time
        else:
            is_blinking = False

        # --- DIRECTION LOGIC (HORIZONTAL PRIORITY) ---
        if not is_blinking and current_state == "IDLE":
            # Deviation from center
            dx = raw_x - center_x
            dy = raw_y - center_y
            
            mag_x = abs(dx)
            mag_y = abs(dy)
            
            new_cmd = None

            # PRIORITY CHECK:
            # We check Horizontal FIRST with a low barrier.
            # We only allow Vertical if it is significantly stronger than horizontal.
            
            # 1. Check Horizontal (The "Wide Cone")
            # Logic: If you moved enough in X, AND your X movement is stronger than 
            # half your Y movement, it's Left/Right.
            if mag_x > SENSITIVITY_X and mag_x > (mag_y * HORIZONTAL_BIAS): 
                if dx < 0: new_cmd = "LEFT"
                else: new_cmd = "RIGHT"
            
            # 2. Check Vertical (The "Narrow Strip")
            # Logic: If we didn't trigger Left/Right, AND we moved enough in Y...
            elif mag_y > SENSITIVITY_Y:
                # Vertical Dominant
                # Note: Y is inverted (Negative = Up/Forward)
                if dy < 0: new_cmd = "FORWARD"
                else: new_cmd = "REVERSE"

            if new_cmd:
                current_state = new_cmd

        # --- VISUALIZATION ---
        color = (0, 0, 255)
        if current_state == "IDLE": color = (0, 255, 255)
        if current_state in ["LEFT", "RIGHT", "FORWARD", "REVERSE"]: color = (0, 255, 0)
        
        # Center Cross
        cx, cy = int(w/2), int(h/2)
        cv2.line(frame, (cx-20, cy), (cx+20, cy), (100, 100, 100), 1)
        cv2.line(frame, (cx, cy-20), (cx, cy+20), (100, 100, 100), 1)
        
        # Gaze Vector
        gx = int(cx + (raw_x - center_x) * 1000)
        gy = int(cy + (raw_y - center_y) * 1000)
        cv2.arrowedLine(frame, (cx, cy), (gx, gy), color, 3)
        
        # UI Text
        cv2.putText(frame, f"CMD: {current_state}", (20, 50), 
                    cv2.FONT_HERSHEY_DUPLEX, 1.5, color, 2)
        
        # Debug Weights
        debug_color = (200, 200, 200)
        cv2.putText(frame, f"X-Mag: {abs(raw_x - center_x):.3f}", (20, h-60), cv2.FONT_HERSHEY_PLAIN, 1, debug_color, 1)
        cv2.putText(frame, f"Y-Mag: {abs(raw_y - center_y):.3f}", (20, h-40), cv2.FONT_HERSHEY_PLAIN, 1, debug_color, 1)

    # Output
    if current_state != last_printed_state:
        print(f"DEVICE_SIGNAL: {current_state}")
        last_printed_state = current_state

    cv2.imshow("Eye Controller", frame)
    if cv2.waitKey(1) & 0xFF == 27: break

cap.release()
cv2.destroyAllWindows()