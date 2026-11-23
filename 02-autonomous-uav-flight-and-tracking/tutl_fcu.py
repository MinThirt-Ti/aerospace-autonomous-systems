import math
from ursina import *

app = Ursina()

# --- 1. SETUP ENVIRONMENT ---
Sky()
ground = Entity(model='plane', scale=50, texture='white_cube', texture_scale=(50,50), color=color.gray)
target = Entity(model='sphere', color=color.red, scale=1, position=(5, 0, 5))

# --- 2. DRONE HIERARCHY ---

# 1. The Controller (The "Real" Drone)
# This handles Position and Yaw. It stays flat (rotation_x=0) so math is easy.
drone = Entity(position=(0, 10, 0)) 

# 2. The Body (Visual Only)
# Rotated 60 degrees to "Look Forward-Down". Parented to drone.
drone_body = Entity(parent=drone, model='cube', color=color.black, scale=1)
drone_body.rotation_x = 60

# 3. Height Line
# Parented to drone. Since drone is flat, (0,-1,0) is truly DOWN.
h_line = Entity(parent=drone, model=Mesh(vertices=[Vec3(0,0,0), Vec3(0,-1,0)], mode='line'), color=color.black)

# 4. Sensor/Frustum
# Parented to drone. Rotated 60 to look down with the body.
# Tip at (0,0,0). Base at Z=1.
sensor_mesh = Mesh(
    vertices=[
        # Tip to Base
        Vec3(0,0,0), Vec3(0.5, 0.5, 1),
        Vec3(0,0,0), Vec3(-0.5, 0.5, 1),
        Vec3(0,0,0), Vec3(0.5, -0.5, 1),
        Vec3(0,0,0), Vec3(-0.5, -0.5, 1),
        # Base Cap
        Vec3(0.5, 0.5, 1), Vec3(-0.5, 0.5, 1),
        Vec3(-0.5, 0.5, 1), Vec3(-0.5, -0.5, 1),
        Vec3(-0.5, -0.5, 1), Vec3(0.5, -0.5, 1),
        Vec3(0.5, -0.5, 1), Vec3(0.5, 0.5, 1),
    ],
    mode='line', static=False, thickness=2
)
sensor = Entity(parent=drone, model=sensor_mesh, color=color.cyan)
sensor.rotation_x = 60


# --- 3. HELPER MATH ---

def check_vision(sensor_ent, target_pos):
    """
    Checks if target is inside the sensor cone.
    Uses the Sensor's local coordinate system (which is looking down).
    """
    # Vector from sensor to target
    vec = target_pos - sensor_ent.world_position
    
    # Project onto Sensor's local axes
    z_dist = vec.dot(sensor_ent.forward) # Distance "Forward" (Down relative to world)
    x_dist = vec.dot(sensor_ent.right)   # Distance "Right"
    y_dist = vec.dot(sensor_ent.up)      # Distance "Up" (which is World Back/Front-ish)

    # 1. Depth Check (Must be in front of lens)
    if z_dist <= 0: return False
    
    # 2. Cone Check
    # Our mesh is 1 unit wide at 1 unit deep (Slope = 0.5)
    slope = 0.5
    limit = z_dist * slope
    
    in_x = abs(x_dist) < limit
    in_y = abs(y_dist) < limit
    
    return in_x and in_y


# --- 4. MAIN LOOP ---
camera.position = (0, 40, -40)
camera.look_at((0,0,0))

yaw_speed = 60
move_speed = 6

def update():
    # A. Target Control
    target.x += (held_keys['d'] - held_keys['a']) * time.dt * 10
    target.z += (held_keys['w'] - held_keys['s']) * time.dt * 10

    # B. Visual Updates
    # Scale Height Line: Since it points 1 unit down, scaling by Y makes it touch ground.
    h_line.scale_y = drone.y
    
    # Scale Sensor: Dynamic Ground Projection
    # We need the distance along the forward vector to the ground (y=0)
    # t = -origin.y / direction.y
    # sensor.forward is the world-space direction vector
    forward_y = sensor.forward.y
    
    if forward_y < -0.1: # Ensure we are looking somewhat down
        dist_to_ground = -drone.y / forward_y
        sensor.scale = (dist_to_ground, dist_to_ground, dist_to_ground)
    else:
        # If looking horizon/up, just set a max render distance
        sensor.scale = (50, 50, 50)

    # C. Logic
    
    # 1. Soft Geofence (Distance from 0,0)
    dist_home = distance_xz(drone.position, Vec3(0,0,0))
    is_out_of_bounds = dist_home > 15
    
    # 2. Vision Check
    can_see = check_vision(sensor, target.position)
    
    if is_out_of_bounds:
        sensor.color = color.red
        # Turn back to center (0,0,0)
        target_angle = math.degrees(math.atan2(0 - drone.x, 0 - drone.z))
        
        # Smooth Yaw
        drone.rotation_y = lerp(drone.rotation_y, target_angle, time.dt * 2)
        drone.position += drone.forward * move_speed * time.dt

    elif can_see:
        sensor.color = color.green
        # Calculate angle to target
        dx = target.x - drone.x
        dz = target.z - drone.z
        target_angle = math.degrees(math.atan2(dx, dz))
        
        # Smooth Yaw to face target
        diff = (target_angle - drone.rotation_y + 180) % 360 - 180
        drone.rotation_y += diff * time.dt * 2
        
        # Move Forward (Standard Drone Axis)
        if distance_xz(drone.position, target.position) > 0.5:
            drone.position += drone.forward * move_speed * time.dt
            
    else:
        # Search (Spin)
        sensor.color = color.cyan
        drone.rotation_y += 30 * time.dt

app.run()