import sys
import pygame 
from ursina import *

# ==========================================
# CONFIGURATION
# ==========================================

AXIS_YAW = 0       
AXIS_THROTTLE = 1  
AXIS_ROLL = 3      
AXIS_PITCH = 4     

INVERT_ROLL = False
INVERT_PITCH = False 
INVERT_THROTTLE = False 
INVERT_YAW = False

# --- PHYSICS TUNING ---
SENSITIVITY_ROTATION = 450
THROTTLE_POWER = 150       
GRAVITY = 20               
DRAG = 0.15                
GROUND_FRICTION = 0.9      

# ==========================================
# SETUP (ORDER FIXED)
# ==========================================

# 1. Initialize Pygame Joystick
pygame.init()
pygame.joystick.init()
joystick = None
if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Controller detected: {joystick.get_name()}")

# 2. Initialize Ursina App FIRST (Prevents 'Window' attribute error)
app = Ursina(background_color=color.black, vsync=True)

# 3. Configure Window AFTER initialization
window.size = (1280, 720) 
window.fullscreen = False
window.borderless = False
window.title = "FPV Drone Sim"

# 4. Set Camera Field of View
camera.fov = 90 # Standard FOV (Fixes fisheye distortion)

# ==========================================
# WIREFRAME WORLD
# ==========================================

# High Visibility Grid Floor
ground = Entity(
    model='plane',
    scale=200,
    color=color.rgb(0, 50, 0), # Dark Green background
    texture='white_cube',      # Wireframe grid texture
    texture_scale=(100, 100),  # Tiling
    collider='box'
)

# Obstacles (Neon Gates)
def create_gate(x, z, rotation=0):
    gate_color = color.lime
    Entity(model='cube', position=(x - 2, 2, z), scale=(0.5, 4, 0.5), 
           color=gate_color, collider='box', rotation_y=rotation)
    Entity(model='cube', position=(x + 2, 2, z), scale=(0.5, 4, 0.5), 
           color=gate_color, collider='box', rotation_y=rotation)
    Entity(model='cube', position=(x, 4, z), scale=(4.5, 0.5, 0.5), 
           color=gate_color, collider='box', rotation_y=rotation)

create_gate(0, 20)
create_gate(20, 10, 45)
create_gate(20, -10, 135)
create_gate(0, -20, 0)
create_gate(-20, -10, -135)
create_gate(-20, 10, -45)

# ==========================================
# DRONE MODEL
# ==========================================

drone = Entity(model='cube', scale=(1, 0.2, 1), color=color.white, y=1)

# Battery (Top/Black)
Entity(parent=drone, model='cube', scale=(0.4, 1.5, 0.6), position=(0, 0.8, 0), color=color.black)

# Feet (Bottom/Cyan)
pad_scale = (0.15, 0.5, 0.15)
pad_color = color.cyan
Entity(parent=drone, model='cube', scale=pad_scale, position=(-0.4, -0.5, 0.4), color=pad_color)
Entity(parent=drone, model='cube', scale=pad_scale, position=(0.4, -0.5, 0.4), color=pad_color)
Entity(parent=drone, model='cube', scale=pad_scale, position=(-0.4, -0.5, -0.4), color=pad_color)
Entity(parent=drone, model='cube', scale=pad_scale, position=(0.4, -0.5, -0.4), color=pad_color)

# Nose (Red) - Fixed Typo
Entity(parent=drone, model='cube', scale=(0.2, 1.5, 0.2), position=(0, 0, 0.6), color=color.red)

# Camera Start Position (FPV)
camera.parent = drone
camera.position = (0, 0.5, 0)
camera.rotation = (0, 0, 0)
drone.visible = False 

velocity = Vec3(0, 0, 0)

# ==========================================
# LOGIC
# ==========================================

def input(key):
    if key == 'v':
        if camera.parent == drone:
            # --- SWITCH TO PILOT VIEW (Stationary) ---
            camera.parent = scene
            # Stand at the "starting line" (Fixed position)
            camera.position = (0, 2, -10)
            camera.look_at(drone)
            drone.visible = True 
        else:
            # --- SWITCH TO FPV VIEW ---
            camera.parent = drone
            camera.position = (0, 0.5, 0)
            camera.rotation = (0, 0, 0)
            drone.visible = False

    if key == 'r':
        drone.position = (0, 1, 0)
        drone.rotation = (0, 0, 0)
        velocity = Vec3(0, 0, 0)
    
    if key == 'escape':
        application.quit()

def update():
    global velocity
    dt = time.dt

    # Inputs
    roll_input, pitch_input, yaw_input, throttle_input = 0, 0, 0, 0

    if joystick:
        pygame.event.pump()
        r = joystick.get_axis(AXIS_ROLL)
        p = joystick.get_axis(AXIS_PITCH)
        t = joystick.get_axis(AXIS_THROTTLE)
        y = joystick.get_axis(AXIS_YAW)

        roll_input = -r if INVERT_ROLL else r
        pitch_input = -p if INVERT_PITCH else p
        yaw_input = -y if INVERT_YAW else y
        raw_throttle = -t if INVERT_THROTTLE else t
        throttle_input = (raw_throttle + 1) / 2 
    else:
        # Keyboard controls
        roll_input = held_keys['d'] - held_keys['a']
        pitch_input = held_keys['w'] - held_keys['s']
        yaw_input = held_keys['e'] - held_keys['q']
        throttle_input = held_keys['space'] + held_keys['left shift']

    # Physics (Quaternion Rotation Fix)
    rotation_step = Vec3(
        pitch_input * SENSITIVITY_ROTATION * dt,
        yaw_input * SENSITIVITY_ROTATION * dt,
        roll_input * SENSITIVITY_ROTATION * dt  
    )
    drone.rotate(rotation_step)

    thrust_vector = drone.up * throttle_input * THROTTLE_POWER
    gravity_vector = Vec3(0, -GRAVITY, 0)
    drag_vector = velocity * DRAG
    
    velocity += (thrust_vector + gravity_vector - drag_vector) * dt
    drone.position += velocity * dt

    # Collision / Ground
    if drone.y < 0.2:
        drone.y = 0.2
        velocity.y = 0
        velocity.x *= GROUND_FRICTION
        velocity.z *= GROUND_FRICTION

    # Update Pilot Camera (Track drone without moving)
    if camera.parent == scene:
        camera.look_at(drone)

app.run()