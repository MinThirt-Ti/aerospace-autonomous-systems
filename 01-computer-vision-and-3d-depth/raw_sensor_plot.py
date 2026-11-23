import socket
import struct
import time
import numpy as np
import matplotlib.pyplot as plt

# --- CONFIG ---
UDP_IP = "0.0.0.0"
UDP_PORT = 5005
HEADER_SIZE = struct.calcsize('!dfffffffff')
CALIB_FILE = "imu_calibration.npz"
SAMPLES = 300  # Adjust for a longer/shorter capture window

def collect_and_plot():
    print(f"Loading calibration from {CALIB_FILE}...")
    try:
        cal = np.load(CALIB_FILE)
        g_bias = cal['gyro_bias']
        h_iron = cal['hard_iron']
        s_iron = cal['soft_iron']
    except FileNotFoundError:
        print("Calibration file not found. Run MOM.py calibration first.")
        return

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    
    print(f"Listening on port {UDP_PORT}... capturing {SAMPLES} samples. Move the IMU around.")
    
    raw_gyros, raw_mags = [], []
    
    while len(raw_gyros) < SAMPLES:
        data, _ = sock.recvfrom(65535)
        if len(data) >= HEADER_SIZE:
            vals = struct.unpack('!dfffffffff', data[:HEADER_SIZE])
            raw_gyros.append(list(vals[4:7]))
            raw_mags.append(list(vals[7:10]))
            
    sock.close()
    print("Capture complete. Processing data...")

    raw_gyros = np.array(raw_gyros)
    raw_mags = np.array(raw_mags)

    # Apply calibration math identical to MOM.py
    corrected_gyros = raw_gyros - g_bias
    corrected_mags = (raw_mags - h_iron) * s_iron

    time_axis = np.arange(SAMPLES)

    # --- FIG 1: GYROSCOPE ---
    fig_g, axs_g = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig_g.suptitle('Gyroscope: Raw vs Bias Corrected', fontsize=14)
    axis_labels = ['X Axis', 'Y Axis', 'Z Axis']

    for i in range(3):
        axs_g[i].plot(time_axis, raw_gyros[:, i], label='Raw', color='red', alpha=0.6)
        axs_g[i].plot(time_axis, corrected_gyros[:, i], label='Corrected', color='blue', alpha=0.8)
        axs_g[i].set_ylabel(axis_labels[i])
        axs_g[i].grid(True, linestyle='--', alpha=0.5)
        axs_g[i].legend(loc='upper right')

    axs_g[2].set_xlabel('Sample Count')
    fig_g.tight_layout()

    # --- FIG 2: MAGNETOMETER ---
    fig_m, axs_m = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig_m.suptitle('Magnetometer: Raw vs Hard/Soft Iron Corrected', fontsize=14)

    for i in range(3):
        axs_m[i].plot(time_axis, raw_mags[:, i], label='Raw', color='red', alpha=0.6)
        axs_m[i].plot(time_axis, corrected_mags[:, i], label='Corrected', color='blue', alpha=0.8)
        axs_m[i].set_ylabel(axis_labels[i])
        axs_m[i].grid(True, linestyle='--', alpha=0.5)
        axs_m[i].legend(loc='upper right')

    axs_m[2].set_xlabel('Sample Count')
    fig_m.tight_layout()

    plt.show()

if __name__ == "__main__":
    collect_and_plot()