import threading
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.action import ActionClient

import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, MoveItErrorCodes
from moveit_msgs.msg import Constraints, PositionConstraint, JointConstraint
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration





REVOLVING_JOINT = "STS3215_03a_Wrist_Roll-v1_Revolute-55"


class CubeGrasper(Node):

    GROUP_NAME = "arm"
    EEF_LINK = "STS3215_03a-v1-4"
    BASE_FRAME = "base_plate_layer1-v5"

    def __init__(self):
        super().__init__("cube_grasper")

        # MoveIt action client
        self.client = ActionClient(self, MoveGroup, "/move_action")
        self.get_logger().info("Waiting for MoveIt...")
        self.client.wait_for_server()
        self.get_logger().info("Connected.")

        # Load YOLO
        self.model = YOLO("best.pt")
        # RealSense setup

        
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        self.pipeline.start(config)

        # --- warm up camera so frames stabilize ---
        for _ in range(30):
            self.pipeline.wait_for_frames()

        self.align = rs.align(rs.stream.color)

        # --- FIX 1: Persistent joint state cache (no repeated subscribe/destroy) ---
        self.latest_joint_state = None
        self.joint_state_lock = threading.Lock()
        self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 10
        )

        # --- FIX 2: Motion runs in a background thread so the camera loop never blocks ---
        self._is_moving = False          # guards against queuing duplicate goals
        self._motion_thread = None
        self._pending_target = None      # (x, y, z) waiting to be executed
        self._target_lock = threading.Lock()

        # --- KEY FIX: spin executor in background so ROS2 callbacks actually fire ---
        self._executor = rclpy.executors.MultiThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(
            target=self._executor.spin, daemon=True
        )
        self._spin_thread.start()


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

    # -------------------------------------------------
    # Persistent joint-state callback
    # -------------------------------------------------

    def _joint_state_cb(self, msg: JointState):
        with self.joint_state_lock:
            self.latest_joint_state = msg

    def _get_joint_value(self, joint_name: str):
        with self.joint_state_lock:
            msg = self.latest_joint_state
        if msg is None:
            return None
        for name, pos in zip(msg.name, msg.position):
            if name == joint_name:
                return pos
        return None

    # -------------------------------------------------
    # MoveIt motion  (called from background thread)
    # -------------------------------------------------

    def _move_background(self, x, y, z):
        try:
            self._is_moving = True
            self._do_move(x, y, z)
        finally:
            self._is_moving = False

    
    def move_arm(self, joints):

        from builtin_interfaces.msg import Duration
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        TRAJ_TOPIC = "/arm_position_controller/joint_trajectory"

        JOINTS = [
            "STS3215_03a-v1_Revolute-45",
            "STS3215_03a-v1-1_Revolute-49",
            "STS3215_03a-v1-2_Revolute-51",
            "STS3215_03a-v1-3_Revolute-53",
            "STS3215_03a_Wrist_Roll-v1_Revolute-55"
        ]

        pub = self.create_publisher(JointTrajectory, TRAJ_TOPIC, 10)

        msg = JointTrajectory()
        msg.joint_names = JOINTS

        point = JointTrajectoryPoint()
        point.positions = joints
        point.time_from_start = Duration(sec=3)

        msg.points.append(point)

        pub.publish(msg)

        self.get_logger().info(f"Moving arm to {joints}")
        return True



    def _do_move(self, x, y, z):
        # Coordinate remap (preserved from original)
        temp = y
        x = x - 0.025
        y = z + 0.07
        z = temp + 0.15

        pose = PoseStamped()
        pose.header.frame_id = self.BASE_FRAME
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.w = 1.0

        req = MotionPlanRequest()
        req.group_name = self.GROUP_NAME
        req.allowed_planning_time = 10.0
        req.num_planning_attempts = 5
        req.max_velocity_scaling_factor = 0.2
        req.max_acceleration_scaling_factor = 0.2
        req.start_state.is_diff = True

        current_value = self._get_joint_value(REVOLVING_JOINT)
        if current_value is None:
            self.get_logger().warning("No joint state yet — skipping move")
            return

        constraint = Constraints()

        jc = JointConstraint()
        jc.joint_name = REVOLVING_JOINT
        jc.position = current_value
        jc.tolerance_above = 0.05
        jc.tolerance_below = 0.05
        jc.weight = 1.0
        constraint.joint_constraints.append(jc)

        pos = PositionConstraint()
        pos.header = pose.header
        pos.link_name = self.EEF_LINK
        pos.weight = 1.0

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.01, 0.01, 0.01]
        pos.constraint_region.primitives.append(box)
        pos.constraint_region.primitive_poses.append(pose.pose)
        constraint.position_constraints.append(pos)

        req.goal_constraints.append(constraint)

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False

        self.get_logger().info(f"Moving to {x:.3f} {y:.3f} {z:.3f}")

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Goal rejected")
            return

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        code = result_future.result().result.error_code.val
        if code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info("SUCCESS")
        else:
            self.get_logger().error(f"Failed with code {code}")

    # -------------------------------------------------
    # Detection loop  (main thread — never blocks)
    # -------------------------------------------------

    def run(self):
        motion_started = False

        while True:
            # If motion started and thread finished -> exit
            import time
            if motion_started and self._motion_thread is not None:
                if not self._motion_thread.is_alive():

                    self.move_arm([0, -1.57, 1.57, 0, 0.0])                    
                    time.sleep(10.5)
                    self.move_arm([-0.1, -2.2, 1.1, 0.9, 0.0])

                    current_ee = self.ee_pose()       # now reflects the real home pose
                    self.move_gripper(0.7)
                    time.sleep(5.5)                   # let gripper settle too
                    self.move_gripper(0.2)
                    time.sleep(3.5)
                    self.move_arm([0, -1.57, 1.57, 0, 0.0])                    


                    print(current_ee)
                    # self._do_move(
                    #     current_ee[0] + 0.025,
                    #     current_ee[2] - 0.2,
                    #     current_ee[1] - 0.07
                    # )
                    # self.move(current_ee[0],current_ee[1],current_ee[2] - 0.12)

                    self.get_logger().info("Motion finished. Exiting.")
                    self.pipeline.stop()
                    cv2.destroyAllWindows()
                    return

            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)

            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())
            height, width, _ = frame.shape

            # If moving, just display frame
            if self._is_moving:
                cv2.putText(
                    frame,
                    "Moving...",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 165, 255),
                    2,
                )

                cv2.imshow("Cube Detection", frame)
                cv2.waitKey(1)
                continue

            results = self.model(frame, verbose=False)
            annotated = results[0].plot()

            depth_intrinsics = (
                depth_frame.profile.as_video_stream_profile().intrinsics
            )

            for box in results[0].boxes:

                cls = int(box.cls[0])
                name = self.model.names[cls]

                if name not in ("Red Box", "Red Cube"):
                    continue

                x1, y1, x2, y2 = box.xyxy[0]
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                depth_values = []

                for dx in range(-5, 6):
                    for dy in range(-5, 6):

                        px = min(max(cx + dx, 0), width - 1)
                        py = min(max(cy + dy, 0), height - 1)

                        d = depth_frame.get_distance(px, py)

                        if 0.05 < d < 0.6:   # reject background
                            depth_values.append(d)

                if not depth_values:
                    continue

                distance = np.median(depth_values)

                X, Y, Z = rs.rs2_deproject_pixel_to_point(
                    depth_intrinsics, [cx, cy], distance
                )

                self.get_logger().info(f"Cube at {X:.3f} {Y:.3f} {Z:.3f}")
                cv2.circle(annotated, (cx, cy), 5, (0, 0, 255), -1)

                if not motion_started:

                    motion_started = True

                    self._motion_thread = threading.Thread(
                        target=self._move_background,
                        args=(X, Y, Z),
                        daemon=True,
                    )

                    self._motion_thread.start()

                    break

            cv2.imshow("Cube Detection", annotated)

            if cv2.waitKey(1) == 27:
                break

        # self.pipeline.stop()
        # cv2.destroyAllWindows()



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
        jc2.tolerance_above = 0.08 # Tightened slightly to ensure it stays visually locked
        jc2.tolerance_below = 0.08
        jc2.weight = 1.0

        constraint.joint_constraints.append(jc2)

        # -------- Position Constraint --------
        pos = PositionConstraint()
        pos.header = pose.header
        pos.link_name = self.EEF_LINK
        pos.weight = 1.0

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.1 , 0.1, 0.1]   

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



    def move_gripper(self, position):

        from builtin_interfaces.msg import Duration
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        TRAJ_TOPIC = "/gripper_controller/joint_trajectory"

        JOINT = ["STS3215_03a-v1-4_Revolute-57"]

        pub = self.create_publisher(JointTrajectory, TRAJ_TOPIC, 10)

        msg = JointTrajectory()
        msg.joint_names = JOINT

        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(sec=1)

        msg.points.append(point)

        # publish several times so controller definitely receives it
        for _ in range(10):
            pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(f"Gripper moving to {position}")
    
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

    def freaky(self):
                    import time
                    self.move_arm([0, -1.57, 1.57, 0, 0.0])                    
                    time.sleep(4.5)
                    self.move_arm([-0.1, -2.2, 1.1, 0.9, 0.0])

                    current_ee = self.ee_pose()       # now reflects the real home pose
                    self.move_gripper(0.7)
                    time.sleep(3.5)                   # let gripper settle too
                    self.move_gripper(0.2)
                    time.sleep(3.5)
                    self.move_arm([0, -1.57, 1.57, 0, 0.0])                    


                    print(current_ee)
                    # self._do_move(
                    #     current_ee[0] + 0.025,
                    #     current_ee[2] - 0.2,
                    #     current_ee[1] - 0.07
                    # )
                    # self.move(current_ee[0],current_ee[1],current_ee[2] - 0.12)

                    self.get_logger().info("Motion finished. Exiting.")
                    self.pipeline.stop()
                    cv2.destroyAllWindows()
                    return
        

    
   

def main():
    rclpy.init()
    node = CubeGrasper()
    try:
        # node.run()

        node.freaky()

        # node.move_arm([0,-1.57,1.57,0,0.0])
        # node.move_gripper(0.5)
    finally:
        node._executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
