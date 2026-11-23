"""
ArUco Target Tracking & Visual Servoing Module

HARDWARE MOUNT & COORDINATE FRAME SPECIFICATION:
------------------------------------------------
1. Physical Mounting:
   The tracking camera is a belly-mounted, downward-facing (nadir) sensor mounted
   upside-down underneath the UAV airframe to provide maximum clearance above
   landing gear and track ground-plane fiducials (ArUco markers) during flight.

2. Optical Roll Correction:
   Due to the physical inverted mount, raw frames are rotated 180 deg via:
     frame = cv2.flip(frame, -1)
   This reorients the optical frame so that image rows and columns align with
   the drone's forward/lateral travel directions on the floor.

3. Axis & Offset Conventions:
   - OpenCV PnP Optical Axis: +Zcam points outward along the camera lens (nadir / toward floor).
   - Inverted Nadir Alignment:
     * After 180 deg flip, image X corresponds to lateral displacement (left/right).
     * Image Y corresponds to longitudinal displacement (drone fore/aft along floor).
   - Offsets:
     * DRONE_OFFSET_X: Lateral offset from drone yaw center (meters).
     * DRONE_OFFSET_Y: Longitudinal forward lever-arm offset along floor track (meters).
     * DRONE_OFFSET_Z: Vertical lever-arm offset below center of gravity (meters).
   - Ground Plane Pitch:
     Extracted from the rotation matrix R[2, 1], R[2, 2] representing camera tilt
     relative to the horizontal floor plane.
"""

import cv2
import numpy as np
import math
import threading
import queue
import time

# --- DRONE CONFIGURATION ---
# Physical offset of the camera from the drone's center of Yaw (in meters)
DRONE_OFFSET_X = 0.0   # e.g., 0 = centered left/right
DRONE_OFFSET_Y = 0.10  # e.g., 0.10 = 10cm forward of center
DRONE_OFFSET_Z = -0.05 # e.g., camera is 5cm below the center

def camera_processing_thread(q, camera_matrix, dist_coeffs, obj_points, circle_3d_points, is_new_api, aruco_dict, parameters):
    """
    Background thread to handle the blocking webcam I/O and heavy detection math.
    """
    cap = cv2.VideoCapture(0)
    
    # Optional: Force higher framerate/resolution if your webcam supports it
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    # cap.set(cv2.CAP_PROP_FPS, 60)

    while True:
        ret, frame = cap.read()
        if not ret: 
            continue
            
        # HANDLE MOUNTING: Flip frame across both axes to correct for upside-down mount
        # -1 means flipping around both x and y axes (180 degree rotation)
        frame = cv2.flip(frame, -1)
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect markers
        if is_new_api:
            corners, ids, rejected = detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        render_data = None

        if ids is not None and len(ids) > 0:
            ids_flat = ids.flatten()
            
            # Prioritize [39], then [30], then whatever is first
            best_idx = 0 
            if 39 in ids_flat: best_idx = np.where(ids_flat == 39)[0][0]
            elif 30 in ids_flat: best_idx = np.where(ids_flat == 30)[0][0]
            
            best_corners = corners[best_idx][0]
            best_id = ids_flat[best_idx]

            success, rvec, tvec = cv2.solvePnP(obj_points, best_corners, camera_matrix, dist_coeffs)

            if success:
                # 1. Camera to ArUco relative distances
                cam_x, cam_y, cam_z = tvec.flatten()
                
                # 2. Drone Center to ArUco relative distances (Applying offsets)
                drone_x = cam_x + DRONE_OFFSET_X
                drone_y = cam_y + DRONE_OFFSET_Y
                drone_z = cam_z + DRONE_OFFSET_Z
                drone_abs_dist = math.sqrt(drone_x**2 + drone_y**2 + drone_z**2)

                # 3. Extract unmeasurable Pitch from the ground plane
                # Convert rotation vector to 3x3 rotation matrix
                R, _ = cv2.Rodrigues(rvec)
                # Pitch can be derived from the rotation matrix assuming ArUco is flat on the floor
                pitch_rad = math.atan2(R[2, 1], R[2, 2])
                pitch_deg = math.degrees(pitch_rad)

                # Prep data for the display thread
                top_left = best_corners[0].astype(int)
                center = np.mean(best_corners, axis=0).astype(int)
                
                circle_2d, _ = cv2.projectPoints(circle_3d_points, rvec, tvec, camera_matrix, dist_coeffs)
                circle_2d = np.int32(circle_2d).reshape(-1, 2)

                render_data = {
                    "id": best_id,
                    "top_left": tuple(top_left),
                    "center": tuple(center),
                    "ellipse": circle_2d,
                    "drone_x": drone_x,
                    "drone_y": drone_y,
                    "drone_z": drone_z,
                    "abs_dist": drone_abs_dist,
                    "cam_pitch": pitch_deg
                }

        # Push to queue (drop oldest if full to prevent lag buildup)
        if q.full():
            try: q.get_nowait()
            except queue.Empty: pass
            
        q.put((frame, render_data))

def main():
    # 1. Setup Camera Intrinsics
    # NOTE: You MUST replace this with cv2.calibrateCamera results for a real drone.
    # This is still the "dirty" fallback.
    cap_temp = cv2.VideoCapture(0)
    ret, frame_temp = cap_temp.read()
    cap_temp.release()
    
    if not ret:
        print("Failed to grab webcam.")
        return
        
    h, w = frame_temp.shape[:2]
    focal_length = w
    center_x, center_y = w / 2, h / 2
    camera_matrix = np.array([
        [focal_length, 0, center_x],
        [0, focal_length, center_y],
        [0, 0, 1]
    ], dtype=float)
    dist_coeffs = np.zeros((4, 1)) 

    # 2. Setup ArUco 
    try:
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        global detector # Needed for thread scoping in new API
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        is_new_api = True
    except AttributeError:
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters_create()
        is_new_api = False

    # 3. Define 3D points
    marker_size = 0.058 
    half_size = marker_size / 2.0
    obj_points = np.array([
        [-half_size, -half_size, 0],
        [ half_size, -half_size, 0],
        [ half_size,  half_size, 0],
        [-half_size,  half_size, 0]
    ], dtype=np.float32)

    circle_radius = half_size * math.sqrt(2) 
    circle_3d_points = np.array([
        [circle_radius * math.cos(math.radians(a)), circle_radius * math.sin(math.radians(a)), 0.0]
        for a in range(0, 360, 5)
    ], dtype=np.float32)

    # 4. Start the background processing thread
    # Maxsize=2 ensures the display thread only ever looks at the most recent frames
    frame_queue = queue.Queue(maxsize=2)
    worker = threading.Thread(
        target=camera_processing_thread, 
        args=(frame_queue, camera_matrix, dist_coeffs, obj_points, circle_3d_points, is_new_api, aruco_dict, parameters),
        daemon=True
    )
    worker.start()

    print("Starting Multithreaded Tracker... Press 'q' to quit.")

    # 5. Main Display Loop (Must run on the main thread for GUI stability)
    while True:
        try:
            # Wait for a frame from the worker thread
            frame, data = frame_queue.get(timeout=1.0)
        except queue.Empty:
            continue

        if data is not None:
            # Draw center (Red) and Top-Left (Green)
            cv2.circle(frame, data["center"], 5, (0, 0, 255), -1)
            cv2.circle(frame, data["top_left"], 5, (0, 255, 0), -1)
            
            # Draw the planar projected ellipse
            cv2.polylines(frame, [data["ellipse"]], isClosed=True, color=(255, 255, 0), thickness=2)

            # Draw ID
            cv2.putText(frame, f"ID: {data['id']}", (data["top_left"][0] - 15, data["top_left"][1] - 15), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA)

            # Draw Drone-Centric Information
            info_text = [
                f"Tgt ID: {data['id']}",
                f"Drone dX: {data['drone_x']:.3f} m",
                f"Drone dY: {data['drone_y']:.3f} m",
                f"Drone dZ: {data['drone_z']:.3f} m",
                f"Abs Dist: {data['abs_dist']:.3f} m",
                f"Est Pitch: {data['cam_pitch']:.1f} deg"
            ]
            
            y_offset = 30
            for text in info_text:
                cv2.putText(frame, text, (20, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 
                            0.6, (0, 255, 0), 2, cv2.LINE_AA)
                y_offset += 25

        cv2.imshow('ArUco Tracker (Multithreaded)', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()