"""
Stage 03: Coordinated UDP Telemetry & Video Blaster (Transmitter)

Broadcasts synchronized 640x480 video frames prefixed with a 44-byte
binary avionics telemetry header (!dfffffffff) to UDP port 5005.
Paired with Stage 04 receiver (04_coordinated_stream.py).
"""

import time
import math
import struct
import socket
import numpy as np
import cv2

# --- NETWORK CONFIGURATION ---
TARGET_IP = "127.0.0.1"  # Target Ground Station IP (e.g. 192.168.1.10)
PORT = 5005
UDP_MAX = 65507

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# --- SENSOR & CAMERA SETUP (With graceful fallbacks) ---
USE_PICAMERA = False
try:
    from picamera2 import Picamera2
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)})
    picam2.configure(config)
    picam2.start()
    USE_PICAMERA = True
    print("Picamera2 initialized at 640x480.")
except Exception:
    print("Picamera2 not available, falling back to cv2.VideoCapture.")
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Optional hardware IMU setup
USE_HARDWARE_IMU = False
try:
    from mpu9250_jmdev.registers import *
    from mpu9250_jmdev.mpu_9250 import MPU9250
    mpu = MPU9250(
        address_ak=AK8963_ADDRESS,
        address_mpu_master=MPU9050_ADDRESS_68,
        bus=1, gfs=GFS_1000, afs=AFS_8G, mfs=AK8963_BIT_16, mode=AK8963_MODE_C100HZ
    )
    mpu.configure()
    USE_HARDWARE_IMU = True
    print("MPU-9250 hardware IMU engaged.")
except Exception:
    print("MPU-9250 hardware not detected. Generating simulated 9-axis avionics telemetry.")

print(f"Coordinated Telemetry Blaster active. Broadcasting to {TARGET_IP}:{PORT}...")

try:
    while True:
        t_mono = time.monotonic()

        # Acquire 9-axis telemetry (ax, ay, az, gx, gy, gz, mx, my, mz)
        if USE_HARDWARE_IMU:
            accel = mpu.readAccelerometerMaster()
            gyro = mpu.readGyroscopeMaster()
            mag = mpu.readMagnetometerMaster()
        else:
            # Synthetic 9-axis avionics telemetry for bench simulation
            t_sin = math.sin(t_mono)
            accel = [0.0, 0.0, 9.81 + 0.1 * t_sin]
            gyro = [0.5 * math.cos(t_mono), -0.3 * t_sin, 0.0]
            mag = [25.0 + 2.0 * t_sin, -12.0, 42.0]

        # Acquire video frame
        if USE_PICAMERA:
            frame = picam2.capture_array()
        else:
            ret, frame = cap.read()
            if not ret or frame is None:
                # Generate synthetic test pattern if physical webcam is unavailable
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(frame, "SIMULATED TELEMETRY FEED", (50, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # Encode JPEG
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ret:
            jpg_data = buffer.tobytes()
            # 44-byte binary header: double (timestamp) + 9 floats (accel, gyro, mag)
            telemetry_header = struct.pack('!dfffffffff', t_mono, *accel, *gyro, *mag)
            packet = telemetry_header + jpg_data

            if len(packet) <= UDP_MAX:
                try:
                    sock.sendto(packet, (TARGET_IP, PORT))
                except OSError:
                    pass

        time.sleep(0.033)  # ~30 Hz broadcast rate

finally:
    print("Shutting down coordinated blaster...")
    if USE_PICAMERA:
        picam2.stop()
    elif 'cap' in locals():
        cap.release()
    sock.close()
