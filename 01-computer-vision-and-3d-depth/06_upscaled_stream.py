import cv2
import socket
import numpy as np
import struct

UDP_IP = "0.0.0.0"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Increase buffer to 4MB to prevent OS-level drops, though we will manual flush
sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4194304) 
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False) # Non-blocking for the flush

HEADER_SIZE = struct.calcsize('!dfffffffff')

print("Receiving 720p stream - Dropping late frames...")

while True:
    data = None
    # 1. FLUSH QUEUE: Grab the most recent packet, discard the rest
    while True:
        try:
            packet, addr = sock.recvfrom(65535)
            data = packet 
        except BlockingIOError:
            break

    if data is None or len(data) < HEADER_SIZE:
        if cv2.waitKey(1) & 0xFF == ord('q'): break
        continue
    
    try:
        # 2. PROCESS ONLY THE LATEST
        header = data[:HEADER_SIZE]
        t_mono, ax, ay, az, gx, gy, gz, mx, my, mz = struct.unpack('!dfffffffff', header)
        
        frame = cv2.imdecode(np.frombuffer(data[HEADER_SIZE:], dtype=np.uint8), cv2.IMREAD_COLOR)
        
        if frame is not None:
            ts_text = f"MONO: {t_mono:.3f}"
            cv2.putText(frame, ts_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("720p SIT Analysis", frame)
            
    except Exception:
        pass
        
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
sock.close()