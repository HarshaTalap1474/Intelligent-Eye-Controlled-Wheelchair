import cv2
import numpy as np

def detect_eyes(img, classifier):
    gray_frame = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Reducing scaleFactor slightly for better detection on smaller eye regions
    eyes = classifier.detectMultiScale(gray_frame, 1.1, 5)
    width = np.size(img, 1)
    height = np.size(img, 0)
    left_eye = None
    right_eye = None
    
    for (x, y, w, h) in eyes:
        if y + h / 2 < height / 2:
            if x + w / 2 < width / 2:
                left_eye = img[y:y + h, x:x + w]
            else:
                right_eye = img[y:y + h, x:x + w]
    return left_eye, right_eye

def get_gaze_direction(eye_img):
    if eye_img is None or eye_img.size == 0:
        return "STOP", False
    
    # 1. Pre-processing: Convert to gray and remove 'salt and pepper' noise
    gray = cv2.cvtColor(eye_img, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (7, 7), 0)
    
    # 2. Adaptive Thresholding: 
    # Instead of a fixed '55', we find the darkest part of the current eye crop.
    # This helps if lighting changes or if you have dark/light eyes.
    _, threshold = cv2.threshold(gray, 45, 255, cv2.THRESH_BINARY_INV)
    
    # Morphological operations to remove eyelashes/eyebrows from the binary image
    kernel = np.ones((3, 3), np.uint8)
    threshold = cv2.erode(threshold, kernel, iterations=1)
    threshold = cv2.dilate(threshold, kernel, iterations=2)
    threshold = cv2.medianBlur(threshold, 5)

    # DEBUG: Show the processed binary eye. You should see the pupil as a WHITE blob.
    cv2.imshow("Pupil Threshold", threshold)
    
    contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)
    
    if len(contours) > 0:
        cnt = contours[0]
        # Ignore small noise blobs that aren't big enough to be a pupil
        if cv2.contourArea(cnt) < 40:
            return "IDLE", True

        (x, y, w, h) = cv2.boundingRect(cnt)
        eye_w = eye_img.shape[1]
        eye_h = eye_img.shape[0]
        cx, cy = x + w/2, y + h/2
        
        # --- DIRECTIONAL LOGIC ---
        # Horizontal
        if cx < eye_w * 0.35:
            return "RIGHT", True
        elif cx > eye_w * 0.65:
            return "LEFT", True
        
        # Vertical (Forward/Reverse)
        if cy < eye_h * 0.35:
            return "FORWARD", True
        elif cy > eye_h * 0.65:
            return "REVERSE", True
            
        return "IDLE", True
            
    # If no contour is found, it's either a blink or the threshold is wrong
    return "STOP", False