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
LAPTOP_IP = "192.168.1.145"
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

# --- CAMERA SETUP (720p) ---
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (1280, 720)})
picam2.configure(config)
picam2.start()

print(f"Blasting 720p + IMU to {LAPTOP_IP}...")

try:
    while True:
        data_ready.wait(timeout=0.1)
        data_ready.clear()

        # Monotonic time for SIT engineering sync
        t_mono = time.monotonic()
        accel = mpu.readAccelerometerMaster()
        gyro = mpu.readGyroscopeMaster()
        mag = mpu.readMagnetometerMaster()

        frame = picam2.capture_array()

        # Adaptive Encoding for 720p
        quality = 65
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, quality])

        if ret:
            jpg_data = buffer.tobytes()
            telemetry = struct.pack('!dfffffffff', t_mono, *accel, *gyro, *mag)

            # If 720p is too big, loop to reduce quality
            while len(telemetry) + len(jpg_data) > UDP_MAX and quality > 10:
                quality -= 10
                ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
                jpg_data = buffer.tobytes()

            if len(telemetry) + len(jpg_data) <= UDP_MAX:
                try:
                    sock.sendto(telemetry + jpg_data, (LAPTOP_IP, PORT))
                except OSError:
                    pass
finally:
    print("Stopping...")
    picam2.stop()
    lgpio.gpiochip_close(chip)
    print("Stopped")