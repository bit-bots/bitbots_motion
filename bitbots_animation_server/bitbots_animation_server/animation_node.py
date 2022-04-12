#!/usr/bin/env python3
import json

from rclpy.action import ActionClient
import traceback

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from humanoid_league_msgs.action import PlayAnimation
from humanoid_league_msgs.msg import Animation as AnimationMsg
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

from bitbots_animation_server.animation import parse
from sensor_msgs.msg import Imu, JointState
from bitbots_animation_server.resource_manager import find_all_animations_by_name
from humanoid_league_msgs.msg import RobotControlState
from bitbots_animation_server.spline_animator import SplineAnimator
from rclpy.action import ActionServer


class AnimationNode(Node):
    def __init__(self):
        """Starts a simple action server and waits for requests."""
        super().__init__("animation")
        # currently, we set log level to info since the action server is spamming too much
        if not self.get_parameter("use_sim_time"):
            pass  # todo how does this work in rclpy
            # rospy.on_shutdown(self.on_shutdown_hook)
        self.get_logger().debug("Starting Animation Server")
        server = PlayAnimationAction(self.get_name(), self)
        rclpy.spin(self)

    def on_shutdown_hook(self):
        # we got external shutdown, let's still wait a bit, since we probably want to do a shutdown animation
        self.get_clock().sleep_for(Duration(seconds=5))


class PlayAnimationAction(object):
    _feedback = PlayAnimation.Feedback
    _result = PlayAnimation.Result

    def __init__(self, name, node: Node):
        self.node = node
        self.current_joint_states = None
        self.action_name = name
        self.hcm_state = 0
        self.current_animation = None
        self.animation_cache = {}
        all_animations = find_all_animations_by_name(node)
        for animation_name, animation_file in all_animations.items():
            try:
                with open(animation_file) as fp:
                    self.animation_cache[animation_name] = parse(json.load(fp))
            except IOError:
                self.node.get_logger().error("Animation '%s' could not be loaded" % animation_name)
            except ValueError:
                self.node.get_logger().error(
                    "Animation '%s' had a ValueError. Probably there is a syntax error in the animation file. "
                    "See traceback" % animation_name)
                traceback.print_exc()

        # predefined messages for performance
        self.anim_msg = AnimationMsg()
        # AnimationMsg takes a JointTrajectory message to also be able to process trajectories. To keep this
        # functionality, we use this message type, even though we only need a single joint goal in this case.
        self.traj_msg = JointTrajectory()
        self.traj_point = JointTrajectoryPoint()

        node.create_subscription(JointState, "joint_states", self.update_current_pose, 1)
        node.create_subscription(RobotControlState, "robot_state", self.update_hcm_state, 1)
        self.hcm_publisher = node.create_publisher(AnimationMsg, "animation", 1)

        self._as = ActionServer(self, PlayAnimation, self.action_name, self.execute_cb)

    def execute_cb(self, goal):
        """ This is called, when someone calls the animation action"""
        first = True
        self.current_animation = goal.animation

        # publish info to the console for the user
        self.node.get_logger().info("Request to play animation %s", goal.animation)

        if self.hcm_state != 0 and not goal.hcm:  # 0 means controllable
            # we cant play an animation right now
            # but we send a request, so that we may can soon
            self.send_animation_request()
            self.node.get_logger().info("HCM not controllable. Only sent request to make it come controllable.")
            goal.abborted(text="HCM not controllable. Will now become controllable. Try again later.")
            return

        animator = self.get_animation_splines(self.current_animation)
        # start animation
        rate = self.node.create_rate(500)

        while rclpy.ok() and animator:
            # first check if we have another goal
            self.check_for_new_goal()
            new_goal = self._as.current_goal.goal.goal.animation
            # if there is a new goal, calculate new splines and reset the time
            if new_goal != self.current_animation:
                self.current_animation = new_goal
                animator = self.get_animation_splines(self.current_animation)
                first = True

            # if we're here we want to play the next keyframe, cause there is no other goal
            # compute next pose
            t = float(self.node.get_clock().now().seconds_nanoseconds()[0] +
                      self.node.get_clock().now().seconds_nanoseconds()[1] / 1e9) - animator.get_start_time()
            pose = animator.get_positions_rad(t)
            if pose is None:
                # see walking node reset

                # animation is finished
                # tell it to the hcm
                self.send_animation(False, True, goal.hcm, None, None)
                self._as.publish_feedback(PlayAnimation.Feedback(percent_done=100))
                # we give a positive result
                self._as.set_succeeded(PlayAnimation.Result(True))
                return

            self.send_animation(first, False, goal.hcm, pose, animator.get_torque(t))
            first = False  # we have sent the first frame, all frames after this can't be the first
            perc_done = int(((float(self.node.get_clock().now().seconds_nanoseconds()[0] +
                                    self.node.get_clock().now().seconds_nanoseconds()[
                                        1] / 1e9) - animator.get_start_time()) / animator.get_duration()) * 100)
            perc_done = max(0, min(perc_done, 100))
            self._as.publish_feedback(PlayAnimation.Feedback(percent_done=perc_done))

            try:
                # catch exception of moving backwards in time, when restarting simulator
                rate.sleep()
            except:
                self.node.get_logger().warn(
                    "We moved backwards in time. This is probably because the simulation was reset.")

    def get_animation_splines(self, animation_name):
        if animation_name not in self.animation_cache:
            self.node.get_logger().error("Animation '%s' not found" % animation_name)
            self._as.set_aborted(False, "Animation not found")
            return

        parsed_animation = self.animation_cache[animation_name]
        return SplineAnimator(parsed_animation, self.current_joint_states, self.node.get_logger(),
                              self.node.get_clock())

    def check_for_new_goal(self):
        if self._as.is_new_goal_available():
            next_goal = self._as.next_goal
            if not next_goal or not next_goal.get_goal():
                return
            self.node.get_logger().debug("New goal: " + next_goal.get_goal().animation)
            if next_goal.get_goal().hcm:
                self.node.get_logger().debug("Accepted hcm animation %s", next_goal.get_goal().animation)
                # cancel old stuff and restart
                self._as.current_goal.set_aborted()
                self._as.accept_new_goal()
            else:
                # can't run this animation now
                self._as.next_goal.set_rejected()
                # delete the next goal to make sure, that we can accept something else
                self._as.next_goal = None
                self.node.get_logger().warn("Couldn't start non hcm animation because another one is already running.")

    def update_current_pose(self, msg):
        """Gets the current motor positions and updates the representing pose accordingly."""
        self.current_joint_states = msg

    def update_hcm_state(self, msg):
        self.hcm_state = msg.state

    def send_animation_request(self):
        self.anim_msg.request = True
        self.anim_msg.header.stamp = self.node.get_clock().now()
        self.hcm_publisher.publish(self.anim_msg)

    def send_animation(self, first, last, hcm, pose, torque):
        self.anim_msg.request = False
        self.anim_msg.first = first
        self.anim_msg.last = last
        self.anim_msg.hcm = hcm
        if pose is not None:
            self.traj_msg.joint_names = []
            self.traj_msg.points = [JointTrajectoryPoint()]
            # We are only using a single point in the trajectory message, since we only want to send a single joint goal
            self.traj_msg.points[0].positions = []
            self.traj_msg.points[0].effort = []
            for joint in pose:
                self.traj_msg.joint_names.append(joint)
                self.traj_msg.points[0].positions.append(pose[joint])
                if torque:
                    # 1 and 2 should be mapped to 1
                    self.traj_msg.points[0].effort.append(np.clip((torque[joint]), 0, 1))
            self.anim_msg.position = self.traj_msg
        self.anim_msg.header.stamp = self.node.get_clock().now()
        self.hcm_publisher.publish(self.anim_msg)


def main(args=None):
    rclpy.init(args=args)
    animation = AnimationNode()