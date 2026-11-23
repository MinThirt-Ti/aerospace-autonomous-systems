import cv2
import time

# Use V4L2 backend for stability
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

# Kill Autofocus
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0) # 0 to disable
cap.set(cv2.CAP_PROP_FOCUS, 0)     # Set focus value

# Reverting to 480p for high-speed processing
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Tracking configuration
# Adjust these to move your "Ground Center" based on the camera tilt
TARGET_Y_OFFSET = 80 
# Size of the "Deadzone" box (in pixels)
BOX_W, BOX_H = 100, 100 

prev_time = 0

while True:
    ret, frame = cap.read()
    if not ret: break

    # Lightweight flip for upside-down mounting
    frame = cv2.flip(frame, -1)

    h, w, _ = frame.shape
    
    # 1. Physical Camera Center (Red Dot)
    cv2.circle(frame, (w//2, h//2), 5, (0, 0, 255), -1)

    # 2. Tracking Center (shifted for tilt)
    t_center_x = w // 2
    t_center_y = (h // 2) + TARGET_Y_OFFSET
    
    # 3. Threshold / Deadzone Box
    # If the object is inside this box, the drone stays still.
    top_left = (t_center_x - BOX_W//2, t_center_y - BOX_H//2)
    bottom_right = (t_center_x + BOX_W//2, t_center_y + BOX_H//2)
    cv2.rectangle(frame, top_left, bottom_right, (255, 255, 0), 2) # Cyan Box
    
    # 4. Crosshair for alignment
    cv2.line(frame, (t_center_x - 10, t_center_y), (t_center_x + 10, t_center_y), (0, 255, 0), 2)
    cv2.line(frame, (t_center_x, t_center_y - 10), (t_center_x, t_center_y + 10), (0, 255, 0), 2)

    # Performance Monitoring
    curr_time = time.time()
    fps = 1 / (curr_time - prev_time) if (curr_time - prev_time) > 0 else 0
    prev_time = curr_time
    cv2.putText(frame, f"FPS: {int(fps)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow('UAV Tracking Calib', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
