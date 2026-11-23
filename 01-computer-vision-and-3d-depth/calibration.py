import cv2
import numpy as np
import socket
import struct
import os

# --- CONFIG ---
CHECKERBOARD = (9, 6)  
SQUARE_SIZE = 25.0     
UDP_IP = "0.0.0.0"
UDP_PORT = 5005
HEADER_SIZE = struct.calcsize('!dfffffffff')

# --- PREP OBJECT POINTS ---
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

objpoints = [] 
imgpoints = [] 

# --- NETWORK SETUP ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Set a massive buffer so we don't drop packets before we can purge them
sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2097152) 
# Set non-blocking so we can purge the buffer instantly
sock.setblocking(False) 
sock.bind((UDP_IP, UDP_PORT))

print(f"Calibration Tool Engaged. Listening on {UDP_PORT}...")
print(f"Targeting Board: {CHECKERBOARD[0]}x{CHECKERBOARD[1]} internal corners.")
print("Instructions:")
print("[S] - Save current frame for calibration")
print("[C] - Calculate and Save Intrinsics (Need >10 frames)")
print("[Q] - Quit")

captured_count = 0

try:
    while True:
        # --- BUFFER PURGE LOGIC ---
        # Keep reading until the buffer is empty to ensure we get the LATEST frame
        data = None
        while True:
            try:
                packet, _ = sock.recvfrom(65535)
                data = packet
            except BlockingIOError:
                break
        
        if data is None or len(data) < HEADER_SIZE:
            cv2.waitKey(1) # Small sleep to prevent 100% CPU usage
            continue
        
        frame_raw = np.frombuffer(data[HEADER_SIZE:], dtype=np.uint8)
        frame = cv2.imdecode(frame_raw, cv2.IMREAD_COLOR)
        
        if frame is None: continue
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        display_frame = frame.copy()

        # Real-time detection feedback
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, 
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

        if ret:
            cv2.drawChessboardCorners(display_frame, CHECKERBOARD, corners, ret)
            cv2.putText(display_frame, "READY", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            cv2.putText(display_frame, "NO BOARD / WRONG SIZE", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.putText(display_frame, f"Saved: {captured_count}", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
        cv2.imshow("Calibration Feed", display_frame)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('s') and ret:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            objpoints.append(objp)
            imgpoints.append(corners2)
            captured_count += 1
            print(f"Captured point {captured_count}")

        elif key == ord('c'):
            if captured_count < 10:
                print(f"Warning: Only {captured_count} frames. Need at least 10.")
                if captured_count < 3: continue
            
            print("Calculating Intrinsics...")
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
            
            if ret:
                np.savez("calibration_data.npz", mtx=mtx, dist=dist)
                mean_error = 0
                for i in range(len(objpoints)):
                    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
                    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
                    mean_error += error
                
                print("--- CALIBRATION SUCCESS ---")
                print(f"RMS Error: {mean_error/len(objpoints)}")
                print("Camera Matrix (mtx):")
                print(mtx)
                print("Saved to: calibration_data.npz")
            else:
                print("Calibration Failed.")

        elif key == ord('q'):
            break

finally:
    cv2.destroyAllWindows()
    sock.close()