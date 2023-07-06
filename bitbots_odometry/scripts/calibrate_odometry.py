"""
This script is used to calibrate the odometry of the robot.
We walk a certain distance in all directions and measure the odometry manually.
The script then calculates the odometry error for the given robot and saves it to a file.
It uses the robots pathfinding to move the robot in the odometry frame.

ROS 2
"""

import os
import sys
import time
import math
import numpy as np
import rclpy
import tf2_ros as tf2
import ros2_numpy as rnp
import threading
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped, Point, Quaternion, Pose
from nav_msgs.msg import Odometry
from bitbots_tf_listener import TransformListener
from tf_transformations import quaternion_from_euler
from std_msgs.msg import Header


class OdometryCalibration(Node):
    def __init__(self):
        super().__init__("odometry_calibration")
        self.get_logger().info("Starting odometry calibration")

        # Goal poses in the odometry frame
        # ("Name", "x", "y", "yaw (deg)")
        self.goal_poses = [
            ("forward", 1.0, 0.0, 0.0),
            ("backward", -1.0, 0.0, 0.0),
            ("left", 0.0, 1.0, 0.0),
            ("right", 0.0, -1.0, 0.0),
            ("forward_left", 1.0, 1.0, 0.0),
            ("forward_right", 1.0, -1.0, 0.0),
            ("backward_left", -1.0, 1.0, 0.0),
            ("backward_right", -1.0, -1.0, 0.0),
            ("turn_left", 0.0, 0.0, 90.0),
            ("turn_right", 0.0, 0.0, -90.0),
            ("turn_around", 0.0, 0.0, 180.0),
            ("forward_90_left", 1.0, 0.0, 90.0),
            ("forward_90_right", 1.0, 0.0, -90.0),
        ]

        # Get current odometry pose as start pose (via tf)
        self.start_pose = None
        self.get_logger().info("Waiting for odometry pose")
        self.tf_buffer = tf2.Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Wait for odometry pose
        while self.start_pose is None:
            try:
                self.start_pose = self.tf_buffer.lookup_transform("base_footprint", "odom", Time())
            except Exception as e:
                self.get_logger().info("Waiting for odometry pose")
                time.sleep(0.1)

        self.get_logger().info("Got odometry pose")

        # Create publisher for goal pose
        self.goal_pose_pub = self.create_publisher(PoseStamped, 'goal_pose', 1)

        # Goal reached threshold
        self.goal_reached_threshold = 0.1

        # Define the current goal pose
        self.current_goal_pose = None

        # Scaling factor buffer
        self.x_scale_factor = []
        self.y_scale_factor = []
        self.yaw_scale_factor = []

        # Spawn ui thread to wait for user input
        threading.Thread(target=self.input_thread, daemon=True).start()

    def input_thread(self):
        # Wait for user input
        input("Press enter to start calibration sequence")

        for goal_pose in self.goal_poses:
            # Set current goal pose
            self.current_goal_pose = goal_pose

            # Tell the user that he should move the robot to the start point and press enter
            self.get_logger().info(
                "Move the robot to the start point and press enter.\n"
                "It might be usefull to mark the start point on the ground for later reference.\n"
                f"Performing task '{self.current_goal_pose[0]}'.\n"
                f"Our current goal pose (relative to the start pose) is: {self.current_goal_pose}"
            )

            # Ask user to press enter
            input("Press enter to continue. The robot will move to the goal pose now.")

            # Calculate goal pose in the odometry frame
            x, y, z, w = quaternion_from_euler(0, 0, math.radians(self.current_goal_pose[3]))

            # Make goal pose message (in the base_footprint frame because we want to move relative to the start pose we are currently in)
            goal_pose_msg = PoseStamped(
                header=Header(
                    stamp=Time(),
                    frame_id="base_footprint"
                ),
                pose=Pose(
                    position=Point(
                        x=self.current_goal_pose[1],
                        y=self.current_goal_pose[2],
                        z=0.0
                    ),
                    orientation=Quaternion(x=x, y=y, z=z, w=w)
                )
            )

            # Transform goal pose to the odometry frame
            goal_pose_msg = self.tf_buffer.transform(goal_pose_msg, "odom", Duration(seconds=1))

            # Publish goal pose
            self.goal_pose_pub.publish(goal_pose_msg)

            # Wait until the goal pose is reached
            while self.current_goal_pose is not None:
                # Check if goal pose is reached
                if self.is_goal_pose_reached():
                    # Goal pose reached
                    self.get_logger().info("Goal pose reached")
                    self.current_goal_pose = None

            # Prompt user to measure odometry
            self.get_logger().info("Please measure the odometry as seen from the start point and press enter when you are done.")

            # Ask for the user to enter the measured odometry
            self.get_logger().info("Please enter the measured odometry (in meters/degrees): ")

            measured_x = float(input("x: "))
            measured_y = float(input("y: "))
            measured_yaw = math.radians(float(input("yaw (deg) (clockwise is negative): ")))

            # Calculate the odometry error
            self.get_logger().info("Calculating odometry error")

            # Estimate the x, y, yaw scale factor
            x_scale_factor = measured_x / self.current_goal_pose[1]
            y_scale_factor = measured_y / self.current_goal_pose[2]
            yaw_scale_factor = measured_yaw / math.radians(self.current_goal_pose[3])

            # Add the scale factors to the buffer
            self.x_scale_factor.append(x_scale_factor)
            self.y_scale_factor.append(y_scale_factor)
            self.yaw_scale_factor.append(yaw_scale_factor)

            # Print the scale factors
            self.get_logger().info(f"The current average scale factors are:\n"
                                   f"x scale factor: {np.mean(self.x_scale_factor)} | "
                                   f"y scale factor: {np.mean(self.y_scale_factor)} | "
                                   f"yaw scale factor: {np.mean(self.yaw_scale_factor)}")

        # Tell the user that the calibration is done
        self.get_logger().info("Calibration done")
