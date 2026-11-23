import cv2
import numpy as np
import os
import time
import math
import urllib.request
import ssl
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

# --- 0. SSL HACK (Copied from your env) ---
ssl._create_default_https_context = ssl._create_unverified_context

# --- 1. SETUP MODEL ---
FACE_MODEL_URL = "https://storage.googleapis.com/mediapipe-models/face_landmarker/face_landmarker/float16/1/face_landmarker.task"
MODEL_FILENAME = "face_landmarker.task"

def download_model(url, filename):
    if not os.path.exists(filename):
        print(f"Downloading {filename}...")
        try:
            urllib.request.urlretrieve(url, filename)
        except Exception as e:
            print(f"Failed to download model: {e}")
            return False
    return True

# Ensure model exists
download_model(FACE_MODEL_URL, MODEL_FILENAME)

# Configure MediaPipe Task API (Exact match to your video_vision.py style)
base_options = python.BaseOptions(model_asset_path=MODEL_FILENAME)
options = vision.FaceLandmarkerOptions(
    base_options=base_options,
    output_face_blendshapes=False,
    output_facial_transformation_matrixes=False,
    num_faces=1,
    running_mode=vision.RunningMode.VIDEO
)

# Initialize detector
detector = vision.FaceLandmarker.create_from_options(options)

# --- 2. DRONE MATH HELPERS ---
def draw_vector(image, start_point, vector, color=(0, 255, 255), thickness=2):
    """Draws an arrow representing the vector"""
    end_point = (int(start_point[0] + vector[0]), int(start_point[1] + vector[1]))
    cv2.arrowedLine(image, start_point, end_point, color, thickness, tipLength=0.2)

# --- 3. MAIN LOOP ---
def run_drone_tracker(source=0):
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        print("Error: Camera not found.")
        return

    # Set Resolution (Optional, keeps FPS high)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    DEADZONE_RADIUS = 50 # Pixels where drone should hold position

    print("Drone Tracker Active. Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Flip for "Mirror" feel (optional, makes it easier to test interactions)
        frame = cv2.flip(frame, 1)
        
        # Dimensions
        h, w = frame.shape[:2]
        center_x, center_y = w // 2, h // 2

        # Prepare for MediaPipe (RGB + MP Image Format)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
        timestamp_ms = int(time.time() * 1000)

        # Detect
        detection_result = detector.detect_for_video(mp_image, timestamp_ms)
        
        # --- DRAWING & LOGIC ---
        
        # 1. Draw Frame Center (The Drone's "Set Point") - Blue Dot
        cv2.circle(frame, (center_x, center_y), 6, (255, 0, 0), -1)
        
        target_found = False

        if detection_result.face_landmarks:
            target_found = True
            
            # Get the first face
            face = detection_result.face_landmarks[0]
            
            # Use Landmark 1 (Nose Tip) as the tracking target
            nose = face[1] 
            target_x = int(nose.x * w)
            target_y = int(nose.y * h)

            # Draw Target (Nose) - Green Dot
            cv2.circle(frame, (target_x, target_y), 6, (0, 255, 0), -1)

            # --- MATH: CALCULATE VECTORS ---
            
            # Deviation: Vector from Center -> Target
            dev_x = target_x - center_x
            dev_y = target_y - center_y

            # Correction: Vector from Center -> Opposite Direction (PID Error Term)
            raw_corr_x = -dev_x
            raw_corr_y = -dev_y
            
            # Magnitude (Euclidean Distance)
            magnitude = math.sqrt(raw_corr_x**2 + raw_corr_y**2)

            # DEADZONE LOGIC
            in_deadzone = magnitude < DEADZONE_RADIUS
            
            if in_deadzone:
                # Kill commands if inside deadzone
                corr_x, corr_y = 0, 0
                deadzone_color = (0, 255, 0) # Green (Safe/Hold)
                status = "HOLD (DEADZONE)"
            else:
                # Active tracking
                corr_x, corr_y = raw_corr_x, raw_corr_y
                deadzone_color = (0, 0, 255) # Red (Active)
                status = "TRACKING"

            # --- VISUALIZATION ---
            
            # Draw Correction Vector (Yellow Arrow)
            # Only draw if we are actually sending commands
            if not in_deadzone:
                draw_vector(frame, (center_x, center_y), (corr_x, corr_y))

            # Draw Deadzone Circle
            cv2.circle(frame, (center_x, center_y), DEADZONE_RADIUS, deadzone_color, 2)

            # --- DATA READOUT ---
            # Display Raw Data (Simulating MAVLink output)
            lines = [
                f"STATUS: {status}",
                f"Target: ({target_x}, {target_y})",
                f"CMD X: {corr_x} (Yaw)",
                f"CMD Y: {corr_y} (Pitch)",
                f"MAGNITUDE: {magnitude:.1f}"
            ]
            
            for i, line in enumerate(lines):
                cv2.putText(frame, line, (10, 30 + i * 25), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        else:
            # No Target
            cv2.putText(frame, "STATUS: SEARCHING...", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Show Result
        cv2.imshow("Drone Control Vector", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    run_drone_tracker()