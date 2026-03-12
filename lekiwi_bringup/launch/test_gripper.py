#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

TRAJ_TOPIC = "/gripper_controller/joint_trajectory"

JOINTS = [
    "STS3215_03a-v1-4_Revolute-57"
]


class ArmDemo(Node):

    def __init__(self):
        super().__init__("lekiwi_arm_demo")

        self.pub = self.create_publisher(JointTrajectory, TRAJ_TOPIC, 10)

        # Movement sequence (radians)
        self.poses = [
        [0.0],
        [0.5],
        ]

        self.phase = 0

        self.get_logger().info(
            f"Publishing trajectories to {TRAJ_TOPIC}"
        )

        # send new pose every 4 seconds
        self.timer = self.create_timer(4.0, self.tick)

    def tick(self):

        msg = JointTrajectory()
        msg.joint_names = JOINTS

        point = JointTrajectoryPoint()
        point.positions = self.poses[self.phase]
        point.time_from_start = Duration(sec=1)

        msg.points.append(point)

        self.pub.publish(msg)

        self.get_logger().info(
            f"Phase {self.phase} → {self.poses[self.phase]}"
        )

        self.phase = (self.phase + 1) % len(self.poses)


def main():
    rclpy.init()
    node = ArmDemo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()