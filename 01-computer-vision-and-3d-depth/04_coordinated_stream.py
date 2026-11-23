import cv2
import socket
import numpy as np
import struct

UDP_IP = "0.0.0.0"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1048576)
sock.bind((UDP_IP, UDP_PORT))

# !dfffffffff -> 8 + (9 * 4) = 44 bytes
HEADER_SIZE = struct.calcsize('!dfffffffff')

print(f"Listening for 9-Axis Telemetry + Video on {UDP_PORT}...")

while True:
    try:
        data, addr = sock.recvfrom(65535)
        
        if len(data) < HEADER_SIZE:
            continue
            
        # 1. Unpack Header
        header_data = data[:HEADER_SIZE]
        t, ax, ay, az, gx, gy, gz, mx, my, mz = struct.unpack('!dfffffffff', header_data)
        
        # 2. Decode Image
        img_data = data[HEADER_SIZE:]
        np_data = np.frombuffer(img_data, dtype=np.uint8)
        frame = cv2.imdecode(np_data, cv2.IMREAD_COLOR)
        
        if frame is not None:
            # 3. Offboard Processing (RGB Swap)
            # frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Text Overlays for Real-time Monitoring
            imu_text = f"A: {ax:>4.1f} {ay:>4.1f} {az:>4.1f} | G: {gx:>5.0f} {gy:>5.0f} {gz:>5.0f}"
            mag_text = f"M: {mx:>5.1f} {my:>5.1f} {mz:>5.1f} | T: {t % 100:.2f}"
            
            cv2.putText(frame, imu_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(frame, mag_text, (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            
            cv2.imshow("SIT Aerospace - Legion Slim 7 Analysis", frame)
            
    except Exception as e:
        print(f"Frame Error: {e}")
        
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
sock.close()