import numpy as np
import matplotlib.pyplot as plt

# Configuration variables
START = np.array([1.0, 1.0])
GOAL = np.array([9.0, 9.0])
OBS_RADIUS = 3.0
ATT_GAIN = 1.0
REP_GAIN = 50.0
STEP_SIZE = 0.05
MAX_STEPS = 500

# Generate grid environment
grid_size = 20
x = np.linspace(0, 10, grid_size)
y = np.linspace(0, 10, grid_size)
X, Y = np.meshgrid(x, y)

# Setup figure
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)
ax.plot(START[0], START[1], 'go', markersize=10, label='Start')
ax.plot(GOAL[0], GOAL[1], 'r*', markersize=15, label='Goal')

# Initialize plots
quiver_plot = ax.quiver(X, Y, np.zeros_like(X), np.zeros_like(Y))
path_line, = ax.plot([], [], 'b-', linewidth=2, label='Path')

def calculate_grid_forces(obs_pos):
    U_x_att = -ATT_GAIN * (X - GOAL[0])
    U_y_att = -ATT_GAIN * (Y - GOAL[1])

    dist_to_obs = np.sqrt((X - obs_pos[0])**2 + (Y - obs_pos[1])**2)
    dist_to_obs[dist_to_obs == 0] = 0.001

    dx = (X - obs_pos[0]) / dist_to_obs
    dy = (Y - obs_pos[1]) / dist_to_obs

    U_x_rep = np.zeros_like(X)
    U_y_rep = np.zeros_like(Y)

    mask = dist_to_obs < OBS_RADIUS
    rep_mag = REP_GAIN * (1.0 / dist_to_obs[mask] - 1.0 / OBS_RADIUS) * (1.0 / dist_to_obs[mask]**2)
    
    U_x_rep[mask] = rep_mag * dx[mask]
    U_y_rep[mask] = rep_mag * dy[mask]

    F_x = U_x_att + U_x_rep
    F_y = U_y_att + U_y_rep

    magnitude = np.sqrt(F_x**2 + F_y**2) + 0.001
    return F_x / magnitude, F_y / magnitude

def simulate_path(obs_pos):
    current_pos = START.copy()
    path_x = [current_pos[0]]
    path_y = [current_pos[1]]
    
    for _ in range(MAX_STEPS):
        # Stop if close enough to goal
        if np.linalg.norm(current_pos - GOAL) < 0.2:
            break
            
        # Attractive force at current point
        f_x = -ATT_GAIN * (current_pos[0] - GOAL[0])
        f_y = -ATT_GAIN * (current_pos[1] - GOAL[1])
        
        # Repulsive force at current point
        dist = np.linalg.norm(current_pos - obs_pos)
        if 0.001 < dist < OBS_RADIUS:
            rep_mag = REP_GAIN * (1.0 / dist - 1.0 / OBS_RADIUS) * (1.0 / dist**2)
            f_x += rep_mag * (current_pos[0] - obs_pos[0]) / dist
            f_y += rep_mag * (current_pos[1] - obs_pos[1]) / dist
            
        # Kinematic update
        current_pos[0] += f_x * STEP_SIZE
        current_pos[1] += f_y * STEP_SIZE
        
        path_x.append(current_pos[0])
        path_y.append(current_pos[1])
        
    return path_x, path_y

def on_mouse_move(event):
    if event.xdata is None or event.ydata is None:
        return
    
    obs_pos = np.array([event.xdata, event.ydata])
    
    # Update grid vectors
    F_x, F_y = calculate_grid_forces(obs_pos)
    quiver_plot.set_UVC(F_x, F_y)
    
    # Update simulated path
    p_x, p_y = simulate_path(obs_pos)
    path_line.set_data(p_x, p_y)
    
    fig.canvas.draw_idle()

# Run initial calculation
initial_obs = np.array([-10, -10])
initial_F_x, initial_F_y = calculate_grid_forces(initial_obs)
quiver_plot.set_UVC(initial_F_x, initial_F_y)
p_x, p_y = simulate_path(initial_obs)
path_line.set_data(p_x, p_y)

fig.canvas.mpl_connect('motion_notify_event', on_mouse_move)
plt.title("Move cursor over the grid to repel vectors and path")
plt.legend()
plt.show()