#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from mavros_msgs.msg import State, RCIn
import sys
import time

class X500HoverVerify(Node):
    def __init__(self):
        super().__init__('x500_hover_verify')

        # Subscriptions
        self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.create_subscription(RCIn, '/mavros/rc/in', self.rc_callback, 10)

        # Service Clients
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')

        # State tracking
        self.current_state = State()
        self.rc_channels = []
        
        # Logic flags
        self.active_mission = False  # True when we control the drone
        self.shutdown_flag = False   # Triggers immediate exit

    def state_callback(self, msg):
        self.current_state = msg
        
        # LOGIC: Passive Pilot Override Detection
        # If we think we are in the mission (GUIDED), but the FCU reports something else,
        # the Pilot has taken manual control. We must yield immediately.
        if self.active_mission and msg.mode != "GUIDED" and not self.shutdown_flag:
            self.get_logger().warn(f"⚠ PILOT OVERRIDE DETECTED: Mode is {msg.mode}. Release control.")
            self.shutdown_flag = True 

    def rc_callback(self, msg):
        self.rc_channels = msg.channels
        if len(self.rc_channels) < 10: 
            return

        # LOGIC: Active Panic Button (CH10 > 1800)
        # If we see this, we FORCE Loiter.
        if self.active_mission and self.rc_channels[9] > 1800 and not self.shutdown_flag:
            self.get_logger().error("🚨 PANIC SWITCH (CH10) DETECTED! FORCING LOITER.")
            self.trigger_failsafe()

    def set_mode(self, mode_name):
        """Asynchronous service call to set mode"""
        if not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Service /mavros/set_mode not available!")
            return False

        req = SetMode.Request()
        req.custom_mode = mode_name
        self.set_mode_client.call_async(req)
        return True

    def trigger_failsafe(self):
        """Active Failsafe: Force LOITER and kill script"""
        self.set_mode("LOITER")
        self.shutdown_flag = True
        
    def call_service_sync(self, client, request):
        """Helper to call services synchronously (blocking)"""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def is_gate_ready(self):
        """Checks for LOITER mode + Mid Throttle"""
        if len(self.rc_channels) < 3: return False
        thr = self.rc_channels[2]
        return (self.current_state.mode == "LOITER" and 1400 <= thr <= 1600)

def main():
    rclpy.init()
    node = X500HoverVerify()

    print("--- HOVER VERIFICATION GATE ---")
    print("Waiting for MAVROS connection...")
    
    # 1. Connection Wait
    while rclpy.ok() and not node.current_state.connected:
        rclpy.spin_once(node, timeout_sec=0.1)

    print("Connected. PILOT: Set Mode LOITER and Throttle MID (1400-1600).")

    # 2. Gate Check Loop
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.is_gate_ready():
            print("\n✅ GATE PASSED.")
            break
        
        thr = node.rc_channels[2] if len(node.rc_channels) > 2 else 0
        sys.stdout.write(f"\r[GATE] Mode: {node.current_state.mode} | Thr: {thr}   ")
        sys.stdout.flush()

    # 3. Mission Execution
    try:
        # Switch to GUIDED
        print("\nAttempting switch to GUIDED mode...")
        node.set_mode("GUIDED")
        
        # Verify Mode Switch
        timeout_start = time.time()
        while rclpy.ok() and (time.time() - timeout_start < 3.0):
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.current_state.mode == "GUIDED":
                node.active_mission = True
                print("✅ GUIDED MODE CONFIRMED.")
                break
        
        if not node.active_mission:
            print("❌ Failed to enter GUIDED. Check RC switches.")
            raise KeyboardInterrupt

        # Arm Drone
        print("Arming...")
        arm_req = CommandBool.Request(value=True)
        if node.call_service_sync(node.arming_client, arm_req).success:
            print("✅ ARMED.")
            
            # Takeoff to 1.5m
            print("Taking off to 1.5m...")
            to_req = CommandTOL.Request(altitude=1.5, latitude=0.0, longitude=0.0, min_pitch=0.0, yaw=0.0)
            if node.call_service_sync(node.takeoff_client, to_req).success:
                print("🚀 TAKEOFF SENT. Hovering for 20 seconds...")
                
                # HOVER LOOP (The dangerous part)
                start_hover = time.time()
                while rclpy.ok() and (time.time() - start_hover < 20.0):
                    rclpy.spin_once(node, timeout_sec=0.1)
                    
                    # SAFETY CHECK: Did we get killed by a callback?
                    if node.shutdown_flag:
                        break
                        
                    remaining = 20 - int(time.time() - start_hover)
                    sys.stdout.write(f"\r[HOVERING] Time Remaining: {remaining}s | Mode: {node.current_state.mode}   ")
                    sys.stdout.flush()
                
                print("") # Newline
                
                # LANDING (Only if we weren't killed by safety logic)
                if not node.shutdown_flag:
                    print("⏳ Hover complete. Landing...")
                    land_req = CommandTOL.Request(latitude=0.0, longitude=0.0, altitude=0.0, min_pitch=0.0, yaw=0.0)
                    node.call_service_sync(node.land_client, land_req)
                    
                    # Wait for disarm or touchdown
                    while rclpy.ok() and node.current_state.armed:
                        rclpy.spin_once(node, timeout_sec=0.5)
                        if node.shutdown_flag: break
                        print("Waiting for touchdown/disarm...")
                    
                    print("✅ LANDED & DISARMED.")

            else:
                print("❌ Takeoff Failed.")
        else:
            print("❌ Arming Failed.")

    except KeyboardInterrupt:
        print("\nKeyboard Interrupt! Safety Halt.")
        node.trigger_failsafe()

    # Cleanup
    print("\n🛑 TEST ENDED. Exiting rclpy.")
    time.sleep(0.5) # Allow final messages to fly
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
