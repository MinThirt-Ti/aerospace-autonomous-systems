#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from mavros_msgs.msg import State, RCIn, PoseStamped
import sys
import time

class X500Figure8Diagonal(Node):
    def __init__(self):
        super().__init__('x500_fig8_diagonal')
        self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.create_subscription(RCIn, '/mavros/rc/in', self.rc_callback, 10)
        self.pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        
        self.setmode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

        self.connected = False
        self.mode = ""
        self.rc_channels = []
        self.myModeSetting = ""

    def state_callback(self, msg):
        self.connected = msg.connected
        self.mode = msg.mode
        if self.myModeSetting == "GUIDED" and self.mode != "GUIDED":
            self.get_logger().fatal(f"OVERRIDE: {self.mode} detected.")
            self.exit_cleanly()

    def rc_callback(self, msg):
        self.rc_channels = msg.channels
        if len(self.rc_channels) >= 10 and self.myModeSetting == "GUIDED":
            if self.rc_channels[9] > 1800: # CH10 SH Pulse
                self.get_logger().fatal("PANIC PULSE! Exit.")
                self.exit_cleanly()

    def exit_cleanly(self):
        rclpy.shutdown()
        sys.exit(0)

    def send_pose(self, x, y, z):
        p = PoseStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = "map"
        p.pose.position.x, p.pose.position.y, p.pose.position.z = float(x), float(y), float(z)
        self.pos_pub.publish(p)

    def is_gate_ready(self):
        if len(self.rc_channels) < 3: return False
        return (self.mode == "LOITER") and (1400 <= self.rc_channels[2] <= 1600)

    def call_service(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    node = X500Figure8Diagonal()
    while rclpy.ok() and not node.connected: rclpy.spin_once(node, timeout_sec=0.1)
    while rclpy.ok() and not node.is_gate_ready(): rclpy.spin_once(node, timeout_sec=0.1)

    if node.call_service(node.setmode_client, SetMode.Request(custom_mode="GUIDED")).mode_sent:
        node.myModeSetting = "GUIDED"
        if node.call_service(node.arming_client, CommandBool.Request(value=True)).success:
            if node.call_service(node.takeoff_client, CommandTOL.Request(altitude=1.5)).success:
                time.sleep(5)
                # Bow-tie / Diagonal 8 points
                # (0,0) -> (1,1) -> (1,-1) -> (0,0) -> (-1,1) -> (-1,-1) -> (0,0)
                path = [(1.0, 1.0, 1.5), (1.0, -1.0, 1.5), (0.0, 0.0, 1.5), 
                        (-1.0, 1.0, 1.5), (-1.0, -1.0, 1.5), (0.0, 0.0, 1.5)]
                for pt in path:
                    print(f"Target: {pt}")
                    for _ in range(40): # 4 seconds per leg
                        node.send_pose(*pt)
                        rclpy.spin_once(node, timeout_sec=0.1)
                node.call_service(node.setmode_client, SetMode.Request(custom_mode="LAND"))
    rclpy.shutdown()
