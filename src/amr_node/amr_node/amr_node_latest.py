#!/usr/bin/env python3
"""
amr_node.py

Single ROS 2 node that:
  1. Subscribes to /cmd_vel (published by teleop_twist_keyboard),
     converts Twist → CMD <left> <right> over serial.
  2. Reads incoming ODOM <left_ticks> <right_ticks> lines from serial,
     computes /odom (nav_msgs/Odometry) and broadcasts the TF odom→base_link.

Requirements:
  • ROS 2 Humble installed & sourced
  • Python packages: rclpy, geometry_msgs, nav_msgs, tf_transformations, tf2_ros
  • pyserial installed: sudo apt install python3-serial
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import serial
import threading
import math
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster

class AMRNode(Node):
    def __init__(self):
        super().__init__('amr_node')

        # ──────────────────────────────────────────────────────
        # Serial Setup (to Arduino)
        # ──────────────────────────────────────────────────────
        # Adjust port if your Arduino appears as /dev/ttyUSB0 or something else
        serial_port = '/dev/ttyACM0'  #assign correct port
        baud_rate   = 115200    # 9600 also works
        timeout_sec = 0.1       # 

        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=timeout_sec)
            self.get_logger().info(f"Opened serial port {serial_port} @ {baud_rate} baud")
        except Exception as e:
            self.get_logger().error(f"Cannot open serial port {serial_port}: {e}")
            exit(1)

        # ──────────────────────────────────────────────────────
        # ROS 2 Subscriptions & Publishers
        # ──────────────────────────────────────────────────────

        # 1) Subscribe to /cmd_vel
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # 2) Advertise /odom
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # 3) TF broadcaster (odom → base_link)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ──────────────────────────────────────────────────────
        # Internal State for Odometry
        # ──────────────────────────────────────────────────────

        # Robot physical parameters (must match your robot)
        self.wheel_radius = 0.05   # meters
        self.wheel_base   = 0.39   # meters (distance between wheels)
        self.ticks_per_rev_left  = 1000  # left encoder ticks per revolution
        self.ticks_per_rev_right = 1000  # right encoder ticks per revolution

        # Pose (x, y, θ) in the “odom” frame
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Previous encoder tick counts (initialized to None until first ODOM arrives)
        self.prev_left_ticks  = None
        self.prev_right_ticks = None

        # Latest encoder ticks (updated from serial)
        self.curr_left_ticks  = None
        self.curr_right_ticks = None

        # Time of last ODOM processing
        self.last_time = self.get_clock().now()

        # ──────────────────────────────────────────────────────
        # Start a background thread to read serial lines
        # ──────────────────────────────────────────────────────
        self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.read_thread.start()

        self.get_logger().info("AMRNode initialized (teleop + odometry).")

    # ─────────────────────────────────────────────────────────
    # 1) /cmd_vel CALLBACK: convert Twist → “CMD <left> <right>\n”
    # ─────────────────────────────────────────────────────────
    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x       # forward speed (m/s)
        w = msg.angular.z      # angular speed (rad/s)

        # Differential‐drive kinematics:
        L = self.wheel_base
        left  = v - (w * L / 2.0)
        right = v + (w * L / 2.0)

        # Normalize so max(|left|, |right|) ≤  1.0
        m = max(abs(left), abs(right), 1.0)
        left_norm  = left  / m
        right_norm = right / m

        # Invert the signs only if teleop “i” (forward) → moves your robot in opposite direction
        left_norm  = left_norm
        right_norm = right_norm

        line = f"CMD {left_norm:.2f} {right_norm:.2f}\n"
        try:
            self.ser.write(line.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")
            return

        self.get_logger().info(f"Sent → {line.strip()}")

    # ─────────────────────────────────────────────────────────
    # 2) SERIAL READER LOOP (background thread)
    #    Reads incoming lines; if line starts with “ODOM”,
    #    splits into two integers: left_ticks, right_ticks.
    # ─────────────────────────────────────────────────────────
    def read_serial_loop(self):
        buffer = ""
        while True:
            try:
                data = self.ser.read(1).decode('utf-8', errors='ignore')
                if not data:
                    continue

                if data == '\n':
                    # We have a full line in “buffer” now
                    line = buffer.strip()
                    buffer = ""

                    if line.startswith("ODOM"):
                        print(f"[DEBUG] Got raw from Arduino: {line}")  # <— debug print
                        parts = line.split()
                        if len(parts) == 3:
                            try:
                                lt = int(parts[1])
                                rt = int(parts[2])
                                # Update “current” tick counts
                                self.curr_left_ticks  = lt  
                                self.curr_right_ticks = rt  
                            except ValueError:
                                self.get_logger().warn(f"Non-int in ODOM: {line}")
                                # Skip update if parsing failed
                                continue
                        else:
                            self.get_logger().warn(f"Malformed ODOM: {line}")
                            continue

                        # ─── Only call update_odometry() once per complete ODOM line ──────────────
                        self.update_odometry()

                    # If the line doesn’t start with “ODOM,” just ignore it
                else:
                    buffer += data

            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                # ── Small sleep to avoid CPU spinning on repeated errors
                import time
                time.sleep(0.01)  # 10 ms pause on error

    # ─────────────────────────────────────────────────────────
    # 3) UPDATE_ODOMETRY: compute dx, dθ from tick counts,
    #                     update (x,y,θ), publish /odom and TF
    # ─────────────────────────────────────────────────────────
    def update_odometry(self):
    	# ensure TransformStamped is available even when run via ros2 run
        from geometry_msgs.msg import TransformStamped
        # ── If we haven’t yet received any valid tick counts, do nothing
        if self.curr_left_ticks is None or self.curr_right_ticks is None:
            return

        # Ensure we have previous tick values; if not, just store current and exit
        if self.prev_left_ticks is None:
            self.prev_left_ticks  = self.curr_left_ticks
            self.prev_right_ticks = self.curr_right_ticks
            self.last_time = self.get_clock().now()
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            # ── If no time has passed, skip computing again
            return
        self.last_time = now

        # Tick differences
        dl_ticks = self.curr_left_ticks  - self.prev_left_ticks
        dr_ticks = self.curr_right_ticks - self.prev_right_ticks
        self.prev_left_ticks  = self.curr_left_ticks
        self.prev_right_ticks = self.curr_right_ticks

        # Convert ticks → distance in meters: ticks/tpr → revolutions → 2πR × revs
        R = self.wheel_radius
        L = self.wheel_base
        d_l = (dl_ticks / self.ticks_per_rev_left)  * 2.0 * math.pi * R
        d_r = (dr_ticks / self.ticks_per_rev_right) * 2.0 * math.pi * R

        # Center‐displacement and heading change
        d_center = (d_r + d_l) / 2.0
        d_theta  = (d_r - d_l) / L

        # Update pose (x, y, θ) in odom frame using Euler integration
        self.theta += d_theta
        self.x     += d_center * math.cos(self.theta)
        self.y     += d_center * math.sin(self.theta)

        # Build an Odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id    = 'odom'
        odom.child_frame_id     = 'base_footprint'

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientation (quaternion from yaw = self.theta)
        q = quaternion_from_euler(0.0, 0.0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Velocity (approximate)
        odom.twist.twist.linear.x  = d_center / dt
        odom.twist.twist.angular.z = d_theta / dt

        # Publish /odom
        self.odom_pub.publish(odom)

        # Also broadcast TF: odom → base_footprint
        t = TransformStamped()
        t.header.stamp    = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)
        print(f"[DEBUG] Published TF odom → base_footprint: θ={self.theta:.3f}")  # <— debug print

def main(args=None):
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)
    node = AMRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Cleanup
    node.ser.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
