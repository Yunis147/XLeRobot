#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
import math

from pick import CubeGrasper as do_next



WHEEL_TOPIC = "/base_velocity_controller/commands"

class CubeFollower(Node):

    def __init__(self):
        super().__init__("cube_follower")

        self.pub = self.create_publisher(
            Float64MultiArray,
            WHEEL_TOPIC,
            10
        )
        self.done = False

        # Load YOLO
        self.model = YOLO("best.pt")

        # RealSense setup
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)

        self.align = rs.align(rs.stream.color)

        self.get_logger().info("Cube follower started")

        self.timer = self.create_timer(0.1, self.loop)

    # 3-omni kinematics (with your axis fix + reversed wheels)
    def omni_kinematics(self, Vx, Vy, W):
        R = 1.0
 
        theta1 = 0
        theta2 = 2 * math.pi / 3
        theta3 = 4 * math.pi / 3

        w1 = -math.sin(theta1) * Vx + math.cos(theta1) * Vy + R * W
        w2 = -math.sin(theta2) * Vx + math.cos(theta2) * Vy + R * W
        w3 = -math.sin(theta3) * Vx + math.cos(theta3) * Vy + R * W

        # Your wheels are reversed
        return [-w1, -w2, -w3]

    def loop(self):

        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)

        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return

        frame = np.asanyarray(color_frame.get_data())
        height, width, _ = frame.shape

        results = self.model(frame, verbose=False)

        Vx = 0.0
        Vy = 0.0
        W  = 0.0

        found = False

        for box in results[0].boxes:
            cls = int(box.cls[0])
            name = self.model.names[cls]

            if name == "Red Box" or name == "Red Cube":

                x1, y1, x2, y2 = box.xyxy[0]
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                # Average depth
                # depth_values = []
                # for dx in range(-2, 3):
                #     for dy in range(-2, 3):
                #         x = min(max(cx + dx, 0), width - 1)
                #         y = min(max(cy + dy, 0), height - 1)
                #         d = depth_frame.get_distance(x, y)
                #         if d > 0:
                #             depth_values.append(d)

                # if not depth_values:
                #     return

                # distance = sum(depth_values) / len(depth_values)

                depth_values = []

                for dx in range(-5, 6):
                    for dy in range(-5, 6):

                        px = min(max(cx + dx, 0), width - 1)
                        py = min(max(cy + dy, 0), height - 1)

                        d = depth_frame.get_distance(px, py)

                        if d > 0:
                            depth_values.append(d)

                if not depth_values:
                    distance = depth_frame.get_distance(cx, cy)
                else:
                    distance = np.median(depth_values)

                self.get_logger().info(f"distance = {distance:.3f}")

                distance = np.median(depth_values)

                error_x = cx - (width // 2)

                # ---------------- CONTROL LOGIC ----------------

                # Rotate toward cube
                k_turn = 0.005
                W = k_turn * error_x

                # Move forward until 30 
                target_distance = 0.15
                k_forward = 10

                if distance > target_distance:
                    Vx = k_forward * (distance - target_distance)
                else:
                    self.get_logger().info("Target distance reached")

                    stop_msg = Float64MultiArray()
                    stop_msg.data = [0.0, 0.0, 0.0]
                    self.pub.publish(stop_msg)

                    # stop camera
                    self.pipeline.stop()

                    cv2.destroyAllWindows()
                    # stop timer so loop stops running
                    self.timer.cancel()

                    # signal main loop to continue
                    self.done = True

                    return

                # Clamp speeds
                Vx = max(min(Vx, 5.0), -5.0)
                W  = max(min(W, 3.0), -3.0)

                found = True
                break

        if not found:
            Vx = 0.0
            Vy = 0.0
            W  = 0.0

        # Axis correction (your robot is rotated 90°)
        Vx_fixed = Vy
        Vy_fixed = -Vx

        wheel_speeds = self.omni_kinematics(Vx_fixed, Vy_fixed, W)

        msg = Float64MultiArray()
        msg.data = wheel_speeds
        self.pub.publish(msg)

        cv2.imshow("Cube Detection", results[0].plot())
        cv2.waitKey(1)



def main():

    rclpy.init()

    node = CubeFollower()

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node)

        node.destroy_node()

        picker = do_next()
        picker.freaky()

    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()