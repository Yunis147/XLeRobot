# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64MultiArray

# WHEEL_TOPIC = "/base_velocity_controller/commands"

# class WheelDemo(Node):

#     def __init__(self):
#         super().__init__("lekiwi_wheel_demo")

#         self.pub = self.create_publisher(
#             Float64MultiArray,
#             WHEEL_TOPIC,
#             10
#         )

#         # velocities (rad/s)
#         self.commands = [
#             [0.0, 10.0, 0.0],     # stop
#             # [5.0, 0.0, -5.0],     # forward
#             # [-5.0, 0.0, 5.0],  # backward            
#             # [0.0, 0.0, 0.0],     # stop
#             # [-1.3,-10.0,6.0], #left
#             # [1.3,6.0,-6.0], #right
#         ]

#         self.phase = 0

#         self.get_logger().info(
#             f"Publishing wheel velocities to {WHEEL_TOPIC}"
#         )

#         self.timer = self.create_timer(4.0, self.tick)

#     def tick(self):

#         msg = Float64MultiArray()
#         msg.data = self.commands[self.phase]

#         self.pub.publish(msg)

#         self.get_logger().info(
#             f"Phase {self.phase} → {self.commands[self.phase]}"
#         )

#         self.phase = (self.phase + 1) % len(self.commands)


# def main():
#     rclpy.init()
#     node = WheelDemo()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass

#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

WHEEL_TOPIC = "/base_velocity_controller/commands"

class WheelDemo(Node):

    def __init__(self):
        super().__init__("lekiwi_wheel_demo")

        self.pub = self.create_publisher(
            Float64MultiArray,
            WHEEL_TOPIC,
            10
        )

        SPEED = 2.0  # you requested 5.0

        # (Vx, Vy, W)
        self.commands = [
            (SPEED, 0.0, 0.0),    # forward
            (-SPEED, 0.0, 0.0),   # backward
            (0.0, -SPEED, 0.0),   # right
            (0.0, SPEED, 0.0),    # left
            (0.0, 0.0, SPEED),    # rotate left
            (0.0, 0.0, -SPEED),   # rotate right
            (0.0, 0.0, 0.0),      # stop
        ]

        self.phase = 0
        self.timer = self.create_timer(4.0, self.tick)

        self.get_logger().info(f"Publishing to {WHEEL_TOPIC}")

    def omni_kinematics(self, Vx, Vy, W):

        R = 1.0

        theta1 = 0
        theta2 = 2 * math.pi / 3
        theta3 = 4 * math.pi / 3

        w1 = -math.sin(theta1) * Vx + math.cos(theta1) * Vy + R * W
        w2 = -math.sin(theta2) * Vx + math.cos(theta2) * Vy + R * W
        w3 = -math.sin(theta3) * Vx + math.cos(theta3) * Vy + R * W

        # 🔴 Reverse all wheels (your hardware is flipped)
        return [-w1, -w2, -w3]
    def tick(self):

        Vx, Vy, W = self.commands[self.phase]

        # 🔴 Fix axis rotation (swap)
        Vx_fixed = Vy
        Vy_fixed = -Vx

        wheel_speeds = self.omni_kinematics(Vx_fixed, Vy_fixed, W)

        msg = Float64MultiArray()
        msg.data = wheel_speeds

        self.pub.publish(msg)

        self.get_logger().info(
            f"Phase {self.phase} | Vx={Vx}, Vy={Vy}, W={W} "
            f"→ wheels={wheel_speeds}"
        )

        self.phase = (self.phase + 1) % len(self.commands)

def main():
    rclpy.init()
    node = WheelDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()