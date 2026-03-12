#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    MoveItErrorCodes,
)
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import JointConstraint
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

import cv2
from ultralytics import YOLO
model = YOLO("best.pt")
PIXEL_SCALE = 0.0003


class SimpleMove(Node):

    GROUP_NAME = "arm"
    EEF_LINK   = "STS3215_03a-v1-4"
    BASE_FRAME = "base_plate_layer1-v5"

    def __init__(self):
        super().__init__("simple_move")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("Waiting for TF...")

        while not self.tf_buffer.can_transform(
            self.BASE_FRAME,
            self.EEF_LINK,
            rclpy.time.Time(),
            timeout=Duration(seconds=1.0)
        ):
            rclpy.spin_once(self)

        self.get_logger().info("TF ready.")

        self.client = ActionClient(self, MoveGroup, "/move_action")
        self.get_logger().info("Waiting for MoveIt...")
        self.client.wait_for_server()
        self.get_logger().info("Connected.")



    def move(self, x, y, z):
        # change y and z vales depending on the real robot.        
        pose = PoseStamped()
        pose.header.frame_id = self.BASE_FRAME
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        req = MotionPlanRequest()
        req.group_name = self.GROUP_NAME
        # --- CRITICAL FIX: Give the planner much more time to solve the rigid path ---
        req.allowed_planning_time = 30.0 
        req.num_planning_attempts = 20
        # -----------------------------------------------------------------------------
        req.max_velocity_scaling_factor = 0.2
        req.max_acceleration_scaling_factor = 0.2
        req.start_state.is_diff = True

        from moveit_msgs.msg import Constraints
        from moveit_msgs.msg import PositionConstraint
        from moveit_msgs.msg import BoundingVolume

        constraint = Constraints()

        revolving_joint = "STS3215_03a_Wrist_Roll-v1_Revolute-55"
        fifth_joint = "STS3215_03a-v1-3_Revolute-53"

        # Get current joint state
        from rclpy.task import Future            
        future = Future()

        def once_callback(msg):
            if not future.done():
                future.set_result(msg)

        sub = self.create_subscription(JointState, "/joint_states", once_callback, 10)

        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        joint_msg = future.result()
        self.destroy_subscription(sub)

        revolving_value = None
        fifth_value = None

        for name, pos in zip(joint_msg.name, joint_msg.position):
            if name == revolving_joint:
                revolving_value = pos
            if name == fifth_joint:
                fifth_value = pos

        # safety check
        if revolving_value is None or fifth_value is None:
            self.get_logger().error("Joint values not found!")
            return

        # -------- Joint Constraint 1 --------
        jc = JointConstraint()
        jc.joint_name = revolving_joint
        jc.position = revolving_value
        jc.tolerance_above = 0.05 # Tightened slightly to ensure it stays visually locked
        jc.tolerance_below = 0.05
        jc.weight = 1.0

        constraint.joint_constraints.append(jc)

        # -------- Joint Constraint 2 --------
        jc2 = JointConstraint()
        jc2.joint_name = fifth_joint
        jc2.position = fifth_value
        jc2.tolerance_above = 0.05 # Tightened slightly to ensure it stays visually locked
        jc2.tolerance_below = 0.05
        jc2.weight = 1.0

        constraint.joint_constraints.append(jc2)

        # -------- Position Constraint --------
        pos = PositionConstraint()
        pos.header = pose.header
        pos.link_name = self.EEF_LINK
        pos.weight = 1.0

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.08 , 0.08, 0.08]   

        pos.constraint_region.primitives.append(box)
        pos.constraint_region.primitive_poses.append(pose.pose)

        # -------- Combine Goal Constraints (Position ONLY) --------
        pose_constraint = Constraints()
        pose_constraint.position_constraints.append(pos)
        
        req.goal_constraints.append(pose_constraint)

        # -------- Apply Path Constraints (Locks the joints during travel) --------
        req.path_constraints = constraint

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False

        self.get_logger().info(f"Moving to {x}, {y}, {z} ")

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error("Goal rejected")
            return

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        code = result.error_code.val

        if code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info("SUCCESS")
        else:
            self.get_logger().error(f"Failed with code {code}")


    def adjust_gripper (self, x, y, z):

        # change y and z vales depending on the real robot.        
        pose = PoseStamped()
        pose.header.frame_id = self.BASE_FRAME
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        req = MotionPlanRequest()
        req.group_name = self.GROUP_NAME
        # --- CRITICAL FIX: Give the planner much more time to solve the rigid path ---
        req.allowed_planning_time = 30.0 
        req.num_planning_attempts = 20
        # -----------------------------------------------------------------------------
        req.max_velocity_scaling_factor = 0.2
        req.max_acceleration_scaling_factor = 0.2
        req.start_state.is_diff = True

        from moveit_msgs.msg import Constraints
        from moveit_msgs.msg import PositionConstraint
        from moveit_msgs.msg import BoundingVolume

        constraint = Constraints()

        revolving_joint = "STS3215_03a_Wrist_Roll-v1_Revolute-55"
        fifth_joint = "STS3215_03a-v1-3_Revolute-53"

        # Get current joint state
        from rclpy.task import Future            
        future = Future()

        def once_callback(msg):
            if not future.done():
                future.set_result(msg)

        sub = self.create_subscription(JointState, "/joint_states", once_callback, 10)

        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        joint_msg = future.result()
        self.destroy_subscription(sub)

        revolving_value = None
        fifth_value = None

        for name, pos in zip(joint_msg.name, joint_msg.position):
            if name == revolving_joint:
                revolving_value = pos
            if name == fifth_joint:
                fifth_value = pos

        # safety check
        if revolving_value is None or fifth_value is None:
            self.get_logger().error("Joint values not found!")
            return

        # -------- Joint Constraint 1 --------
        jc = JointConstraint()
        jc.joint_name = revolving_joint
        jc.position = revolving_value
        jc.tolerance_above = 0.05 # Tightened slightly to ensure it stays visually locked
        jc.tolerance_below = 0.05
        jc.weight = 1.0

        constraint.joint_constraints.append(jc)

        # -------- Joint Constraint 2 --------
        jc2 = JointConstraint()
        jc2.joint_name = fifth_joint
        jc2.position = fifth_value
        jc2.tolerance_above = 0.05 # Tightened slightly to ensure it stays visually locked
        jc2.tolerance_below = 0.05
        jc2.weight = 1.0

        constraint.joint_constraints.append(jc2)

        # -------- Position Constraint --------
        pos = PositionConstraint()
        pos.header = pose.header
        pos.link_name = self.EEF_LINK
        pos.weight = 1.0

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.08 , 0.08, 0.08]   

        pos.constraint_region.primitives.append(box)
        pos.constraint_region.primitive_poses.append(pose.pose)

        # -------- Combine Goal Constraints (Position ONLY) --------
        pose_constraint = Constraints()
        pose_constraint.position_constraints.append(pos)
        
        req.goal_constraints.append(pose_constraint)

        # -------- Apply Path Constraints (Locks the joints during travel) --------
        req.path_constraints = constraint

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False

        self.get_logger().info(f"Moving to {x}, {y}, {z} ")

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error("Goal rejected")
            return

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        code = result.error_code.val

        if code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info("SUCCESS")
        else:
            self.get_logger().error(f"Failed with code {code}")
    

    def ee_pose(self):

        l1 = None
        rclpy.spin_once(self, timeout_sec=0.5)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.BASE_FRAME,
                self.EEF_LINK,
                rclpy.time.Time(),
                timeout=Duration(seconds=2.0)
            )
            t = transform.transform.translation
            self.get_logger().info(
                f"EE Position -> x:{t.x:.4f}, y:{t.y:.4f}, z:{t.z:.4f}"
            )
            l1 = [t.x, t.y, t.z]
        except Exception as e:
            self.get_logger().error(str(e))
        return l1

    def error_xy(self, camera="/dev/video8"):
        cap = cv2.VideoCapture(camera)

        if not cap.isOpened():
            print("Error: Could not open camera", camera)
            return None, None

        while True:
            ret, frame = cap.read()
            if not ret:
                continue
            h, w, _ = frame.shape
            center_x = w // 2
            center_y = h // 2
            annotated = frame.copy()
            cv2.circle(annotated, (center_x, center_y), 6, (255,0,0), -1)
            results = model(frame, verbose=False)

            for box in results[0].boxes:
                name = model.names[int(box.cls[0])]
                if name not in ("Red Box", "Red Cube"):
                    continue
                x1, y1, x2, y2 = box.xyxy[0]
                x1 = int(x1)
                y1 = int(y1)
                x2 = int(x2)
                y2 = int(y2)

                bx = (x1 + x2) // 2
                by = (y1 + y2) // 2

                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0,0,255), 2)
                cv2.circle(annotated, (bx, by), 5, (0,0,255), -1)

                error_x = bx - center_x
                error_y = by - center_y

                move_x = -error_y * PIXEL_SCALE
                move_y = -error_x * PIXEL_SCALE

                print(
                    f"Pixel error: ({error_x}, {error_y})   "
                    f"Move robot: ({move_x:.4f}, {move_y:.4f}) meters"
                )

                cv2.imshow("cube debug", annotated)

                cap.release()
                cv2.destroyAllWindows()

                current_grip = self.ee_pose()
                curr_x = current_grip[0]
                curr_y = current_grip[1]
                curr_z = current_grip[2]

                self.adjust_gripper(-(curr_x + move_x), curr_y + move_y , curr_z)

                return move_x, move_y
            cv2.imshow("cube debug", annotated)
            if cv2.waitKey(1) == 27:
                break
        cap.release()
        cv2.destroyAllWindows()

        return None, None

  

def main():
    rclpy.init()
    node = SimpleMove()
    # node.move(0.0, 0.35, 0.2)
    node.ee_pose()
    node.error_xy()
    # node.move(0.0, 0.25, 0.10)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
