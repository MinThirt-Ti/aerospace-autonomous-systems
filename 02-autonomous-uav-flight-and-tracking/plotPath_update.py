import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from pymavlink import mavutil
import numpy as np

# Connection: Listening to Mission Planner's UDP Outbound
connection = mavutil.mavlink_connection('udpin:127.0.0.1:14550')

# Data Storage
path = []

# Smoothing factor (0.1–0.3 recommended)
alpha = 0.2

def smooth_point(new, prev):
    return prev + alpha * (new - prev)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
line, = ax.plot([], [], [], 'b-', lw=2)

def update(frame):
    new_data = False
    while True:
        msg = connection.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        if not msg:
            break

        raw_point = np.array([msg.x, msg.y, -msg.z])

        # Apply EMA smoothing
        if not path:
            smoothed = raw_point
        else:
            smoothed = smooth_point(raw_point, np.array(path[-1]))

        path.append(smoothed.tolist())
        new_data = True

    if new_data and len(path) > 1:
        pts = np.array(path)
        line.set_data(pts[:, 0], pts[:, 1])
        line.set_3d_properties(pts[:, 2])

        # Dynamic Scaling
        max_range = np.array([
            pts[:, 0].max() - pts[:, 0].min(),
            pts[:, 1].max() - pts[:, 1].min(),
            pts[:, 2].max() - pts[:, 2].min()
        ]).max() / 2.0

        mid_x = (pts[:, 0].max() + pts[:, 0].min()) * 0.5
        mid_y = (pts[:, 1].max() + pts[:, 1].min()) * 0.5
        mid_z = (pts[:, 2].max() + pts[:, 2].min()) * 0.5

        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)

    return line,

ani = FuncAnimation(fig, update, interval=200, blit=False)

ax.set_xlabel('North (m)')
ax.set_ylabel('East (m)')
ax.set_zlabel('Altitude (m)')

plt.show()
