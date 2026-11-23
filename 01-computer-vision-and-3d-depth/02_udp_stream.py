import cv2
import socket
import numpy as np

UDP_IP = "0.0.0.0"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Increase receive buffer for your Ryzen 9 setup to prevent dropped packets
sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1048576)
sock.bind((UDP_IP, UDP_PORT))

print(f"Direct UDP stream active on {UDP_PORT}...")

while True:
    data, addr = sock.recvfrom(65535)
    np_data = np.frombuffer(data, dtype=np.uint8)
    
    # Decode incoming JPEG
    frame = cv2.imdecode(np_data, cv2.IMREAD_COLOR)
    
    if frame is not None:
        # OFFBOARD CONVERSION: Swap BGR back to RGB if your model needs it
        # or leave as BGR for standard cv2.imshow
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # PROCEED WITH ANALYSIS
        # Using the RGB frame for your engineering/data science models:
        # result = your_scikit_learn_or_custom_model(frame_rgb)

        cv2.imshow("Laptop Side Processing", frame) # imshow expects BGR
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
sock.close()