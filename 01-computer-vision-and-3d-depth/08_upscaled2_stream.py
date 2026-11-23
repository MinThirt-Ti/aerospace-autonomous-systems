import cv2
import socket
import numpy as np
import struct

UDP_IP = "0.0.0.0"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Increased to 4MB Buffer for high-frequency 1080p bursts
sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4194304)
sock.bind((UDP_IP, UDP_PORT))

HEADER_SIZE = struct.calcsize('!dfffffffff')

print("Receiving 1080p Aerospace Stream. Press 'q' to stop.")

while True:
    try:
        data, addr = sock.recvfrom(65535)
        if len(data) < HEADER_SIZE: continue
        
        # Unpack monotonic timestamp and IMU data
        t_mono, ax, ay, az, gx, gy, gz, mx, my, mz = struct.unpack('!dfffffffff', data[:HEADER_SIZE])
        
        # Decode 1080p JPEG
        frame = cv2.imdecode(np.frombuffer(data[HEADER_SIZE:], dtype=np.uint8), cv2.IMREAD_COLOR)
        
        if frame is not None:
            # High-res overlay logic
            ts_text = f"MONO: {t_mono:.4f}s"
            imu_text = f"A: {ax:.1f} {ay:.1f} {az:.1f}"
            
            cv2.putText(frame, ts_text, (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2)
            cv2.putText(frame, imu_text, (30, 110), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            
            # Ryzen 9 can handle complex resizing if window is too large for screen
            cv2.imshow("1080p SIT Aerospace Analysis", frame)
            
    except Exception as e:
        # Suppress occasional corrupt frame errors from UDP drops
        pass
        
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
sock.close()