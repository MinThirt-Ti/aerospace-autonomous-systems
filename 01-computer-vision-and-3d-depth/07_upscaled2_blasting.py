import cv2
import socket
import time
import struct
import lgpio
import threading
from picamera2 import Picamera2
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

# --- CONFIGURATION ---
LAPTOP_IP = "192.168.1.10"
PORT = 5005
INT_PIN = 4 
UDP_MAX = 65507
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# --- IMU SETUP ---
data_ready = threading.Event()
def interrupt_handler(chip, gpio, level, tick):
    data_ready.set()

mpu = MPU9250(
    address_ak=AK8963_ADDRESS,
    address_mpu_master=MPU9050_ADDRESS_68,
    bus=1, gfs=GFS_1000, afs=AFS_8G, mfs=AK8963_BIT_16, mode=AK8963_MODE_C100HZ
)
mpu.configure()
mpu.bus.write_byte_data(mpu.address_mpu_master, 0x38, 0x01)

chip = lgpio.gpiochip_open(0)
lgpio.gpio_claim_alert(chip, INT_PIN, lgpio.RISING_EDGE, lgpio.SET_PULL_DOWN)
cb = lgpio.callback(chip, INT_PIN, lgpio.RISING_EDGE, interrupt_handler)

# --- CAMERA SETUP (1080p) ---
picam2 = Picamera2()
# 1920x1080 is the max for the Zero 2W hardware encoder path
config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (1920, 1080)})
picam2.configure(config)
picam2.start()

print(f"Blasting 1080p + IMU Telemetry to {LAPTOP_IP}...")

try:
    while True:
        # Sync to hardware interrupt
        data_ready.wait(timeout=0.1)
        data_ready.clear()

        t_mono = time.monotonic()
        accel = mpu.readAccelerometerMaster()
        gyro = mpu.readGyroscopeMaster()
        mag = mpu.readMagnetometerMaster()
        
        frame = picam2.capture_array()
        
        # Start with lower quality for 1080p to fit UDP MTU
        quality = 50
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        
        if ret:
            jpg_data = buffer.tobytes()
            # 44-byte telemetry header
            telemetry = struct.pack('!dfffffffff', t_mono, *accel, *gyro, *mag)
            
            # Strict UDP packet size enforcement loop
            while len(telemetry) + len(jpg_data) > UDP_MAX and quality > 5:
                quality -= 5
                ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
                jpg_data = buffer.tobytes()

            if len(telemetry) + len(jpg_data) <= UDP_MAX:
                try:
                    sock.sendto(telemetry + jpg_data, (LAPTOP_IP, PORT))
                except OSError:
                    pass # Network jitter suppression
finally:
    picam2.stop()
    lgpio.gpiochip_close(chip)