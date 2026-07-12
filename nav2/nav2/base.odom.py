#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
import tf2_ros
import math
from st3215 import ST3215

servo = ST3215('/dev/ttyACM0')
TICKS_PER_REV = 4096
SQRT3 = 1.73205

class MotorOdom(Node):
    def __init__(self):
        super().__init__('motor_odom')
        self.wheel_radius, self.robot_radius = 0.05, 0.15
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.prev_1, self.prev_2, self.prev_3 = None, None, None

        self.heading_offset_deg = 120.0

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.timer = self.create_timer(0.033, self.update)

    def cmd_callback(self, msg):
        cmd_x, cmd_y, w = msg.linear.x, msg.linear.y, msg.angular.z

        rad = math.radians(self.heading_offset_deg)
        vx = cmd_x * math.cos(rad) - cmd_y * math.sin(rad)
        vy = cmd_x * math.sin(rad) + cmd_y * math.cos(rad)

        v1 = -(SQRT3/2.0)*vx + 0.5*vy + self.robot_radius*w
        v2 = (SQRT3/2.0)*vx + 0.5*vy + self.robot_radius*w
        v3 = -1.0*vy + self.robot_radius*w
        try:
            for i, v in enumerate([v1, v2, v3], 1): servo.Rotate(i, int(v * 3000))
        except: pass

    def update(self):
        try:
            p1, p2, p3 = servo.ReadPosition(1), servo.ReadPosition(2), servo.ReadPosition(3)
        except: return
        if p1 is None or self.prev_1 is None:
            self.prev_1, self.prev_2, self.prev_3 = p1, p2, p3
            return

        def delta(c, p):
            d = c - p
            if d > 2048: d -= 4096
            elif d < -2048: d += 4096
            return d * (2.0 * math.pi * self.wheel_radius / 4096)

        d1, d2, d3 = delta(p1, self.prev_1), delta(p2, self.prev_2), delta(p3, self.prev_3)
        self.prev_1, self.prev_2, self.prev_3 = p1, p2, p3

        raw_dx, raw_dy = (d2 - d1) / SQRT3, (d1 + d2 - (2.0 * d3)) / 3.0

        rad_inv = math.radians(-self.heading_offset_deg)
        dx = raw_dx * math.cos(rad_inv) - raw_dy * math.sin(rad_inv)
        dy = raw_dx * math.sin(rad_inv) + raw_dy * math.cos(rad_inv)

        dth = (d1 + d2 + d3) / (3.0 * self.robot_radius)
        self.x += dx * math.cos(self.theta) - dy * math.sin(self.theta)
        self.y += dx * math.sin(self.theta) + dy * math.cos(self.theta)
        self.theta += dth

        now = self.get_clock().now().to_msg()
        qz, qw = math.sin(self.theta/2.0), math.cos(self.theta/2.0)

        t1 = TransformStamped()
        t1.header.stamp, t1.header.frame_id, t1.child_frame_id = now, "odom", "base_footprint"
        t1.transform.translation.x, t1.transform.translation.y = float(self.x), float(self.y)
        t1.transform.rotation.z, t1.transform.rotation.w = qz, qw

        t2 = TransformStamped()
        t2.header.stamp, t2.header.frame_id, t2.child_frame_id = now, "base_footprint", "base_link"
        t2.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform([t1, t2])

        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = now, "odom", "base_footprint"
        odom.pose.pose.position.x, odom.pose.pose.position.y = float(self.x), float(self.y)
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
        self.odom_pub.publish(odom)

def main():
    rclpy.init(); node = MotorOdom()
    try: rclpy.spin(node)
    except: rclpy.shutdown()
if __name__ == '__main__': main()

# odom → base_footprint → base_link → laser (laser TF comes from slam.py)
