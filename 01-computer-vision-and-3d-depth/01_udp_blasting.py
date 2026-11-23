import cv2
import socket
from picamera2 import Picamera2

# Configure Network
LAPTOP_IP = "192.168.1.10" 
PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

picam2 = Picamera2()
# Capture in RGB888 directly
config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)})
picam2.configure(config)
picam2.start()

try:
    while True:
        # Capture raw array
        frame = picam2.capture_array()
        
        # Encode RGB array to JPEG directly
        # Note: imencode expects BGR, so encoding RGB here means 
        # it will be "swapped" on the laptop side.
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        
        if ret:
            data = buffer.tobytes()
            if len(data) < 65507:
                sock.sendto(data, (LAPTOP_IP, PORT))
finally:
    picam2.stop()