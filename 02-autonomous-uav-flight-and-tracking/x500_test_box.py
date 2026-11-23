#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from mavros_msgs.msg import State, RCIn
from geometry_msgs.msg import PoseStamped
import sys
import time
import math

class X500BoxMacro(Node):
    def __init__(self):
        super().__init__('x500_box_macro')

        self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.create_subscription(RCIn, '/mavros/rc/in', self.rc_callback, 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, 10)

        self.pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')

        self.current_state = State()
        self.current_pose = PoseStamped()
        self.rc_channels = []

        self.active_mission = False
        self.shutdown_flag = False

    def state_callback(self, msg):
        self.current_state = msg
        if self.active_mission and msg.mode != "GUIDED" and not self.shutdown_flag:
            self.get_logger().warn(f"⚠ PILOT OVERRIDE: Mode {msg.mode}. Releasing control.")
            self.shutdown_flag = True

    def rc_callback(self, msg):
        self.rc_channels = msg.channels
        if len(self.rc_channels) < 10: return

        if self.active_mission and self.rc_channels[9] > 1800 and not self.shutdown_flag:
            self.get_logger().error("🚨 PANIC SWITCH DETECTED. FORCING LOITER.")
            self.trigger_failsafe()

    def pose_callback(self, msg):
        self.current_pose = msg

    def trigger_failsafe(self):
        req = SetMode.Request()
        req.custom_mode = "LOITER"
        self.set_mode_client.call_async(req)
        self.shutdown_flag = True

    def send_pose(self, x, y, z):
        p = PoseStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = "map"
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.position.z = float(z)
        p.pose.orientation.w = 1.0
        self.pos_pub.publish(p)

    def get_distance(self, target):
        curr = self.current_pose.pose.position
        return math.sqrt((target[0] - curr.x)**2 + (target[1] - curr.y)**2 + (target[2] - curr.z)**2)

    def is_gate_ready(self):
        if len(self.rc_channels) < 3: return False
        thr = self.rc_channels[2]
        return (self.current_state.mode == "LOITER" and 1400 <= thr <= 1600)

    def call_service_sync(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    node = X500BoxMacro()

    print("--- BOX MACRO MISSION ---")
    print("Waiting for MAVROS connection...")
    while rclpy.ok() and not node.current_state.connected:
        rclpy.spin_once(node, timeout_sec=0.1)

    print("Connected. PILOT: Set Mode LOITER and Throttle MID (1400-1600).")
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.is_gate_ready():
            print("\n✅ GATE PASSED.")
            break
        thr = node.rc_channels[2] if len(node.rc_channels) > 2 else 0
        sys.stdout.write(f"\r[GATE] Mode: {node.current_state.mode} | Thr: {thr}   ")
        sys.stdout.flush()

    try:
        print("\nAttempting switch to GUIDED mode...")
        sm_req = SetMode.Request(custom_mode="GUIDED")
        if node.call_service_sync(node.set_mode_client, sm_req).mode_sent:

            start_wait = time.time()
            while rclpy.ok() and (time.time() - start_wait < 3.0):
                rclpy.spin_once(node, timeout_sec=0.1)
                if node.current_state.mode == "GUIDED":
                    node.active_mission = True
                    print("✅ GUIDED MODE CONFIRMED.")
                    break

            if not node.active_mission:
                print("❌ Failed to enter GUIDED.")
                raise KeyboardInterrupt

            print("Arming...")
            if node.call_service_sync(node.arming_client, CommandBool.Request(value=True)).success:
                print("✅ ARMED.")
                print("Taking off...")
                to_req = CommandTOL.Request(altitude=1.5, latitude=0.0, longitude=0.0, min_pitch=0.0, yaw=0.0)
                if node.call_service_sync(node.takeoff_client, to_req).success:
                    print("🚀 TAKEOFF SENT. Holding 5s...")

                    hold_start = time.time()
                    while rclpy.ok() and (time.time() - hold_start < 5.0):
                        rclpy.spin_once(node, timeout_sec=0.1)
                        if node.shutdown_flag: break

                    if not node.shutdown_flag:
                        points = [
                            (1.5, 0.0, 1.5),
                            (1.5, 1.5, 1.5),
                            (0.0, 1.5, 1.5),
                            (0.0, 0.0, 1.5)
                        ]

                        for i, pt in enumerate(points):
                            if node.shutdown_flag: break
                            print(f"\n📍 Targeting Waypoint {i+1}: {pt}")

                            wp_start_time = time.time()
                            reached = False
                            
                            while rclpy.ok() and not reached:
                                rclpy.spin_once(node, timeout_sec=0.1)
                                if node.shutdown_flag: break

                                dist = node.get_distance(pt)
                                elapsed = time.time() - wp_start_time
                                node.send_pose(*pt)
                                
                                sys.stdout.write(f"\r[FLYING] ✈ >> {pt} | Dist: {dist:.2f}m | Time: {elapsed:.1f}s ")
                                sys.stdout.flush()

                                if dist < 0.3:
                                    reached = True
                                    print(f"\n✅ Waypoint {i+1} Reached.")
                                
                                if elapsed > 15.0:
                                    print(f"\n⌛ Waypoint {i+1} Timeout - Proceeding.")
                                    reached = True

                        if not node.shutdown_flag:
                            print("\n✅ Box Complete. Landing...")
                            land_req = CommandTOL.Request(latitude=0.0, longitude=0.0, altitude=0.0, min_pitch=0.0, yaw=0.0)
                            node.call_service_sync(node.land_client, land_req)

                            while rclpy.ok() and node.current_state.armed:
                                rclpy.spin_once(node, timeout_sec=0.5)
                                if node.shutdown_flag: break
                                print("Waiting for touchdown...")
                            print("✅ LANDED & DISARMED.")
                else:
                    print("❌ Takeoff Failed.")
            else:
                print("❌ Arming Failed.")

    except KeyboardInterrupt:
        node.trigger_failsafe()

    print("\n🛑 TEST ENDED. Exiting rclpy.")
    time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
