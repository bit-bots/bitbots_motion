#!/usr/bin/env python3
import json

import actionlib
import traceback

import numpy as np
import rospy

from humanoid_league_msgs.msg import PlayAnimationResult, PlayAnimationFeedback
from humanoid_league_msgs.msg import PlayAnimationAction as PlayAction
from humanoid_league_msgs.msg import Animation as AnimationMsg
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

from bitbots_animation_server.animation import parse
from sensor_msgs.msg import Imu, JointState
from bitbots_animation_server.resource_manager import find_all_animations_by_name
from humanoid_league_msgs.msg import RobotControlState
from bitbots_animation_server.spline_animator import SplineAnimator

class AnimationNode:
    def __init__(self):
        """Starts a simple action server and waits for requests."""
        # currently we set log level to info since the action server is spamming too much
        log_level = rospy.INFO if rospy.get_param("debug_active", False) else rospy.INFO
        rospy.init_node("animation", log_level=log_level, anonymous=False)
        if not rospy.get_param("simulation_active"):
            rospy.on_shutdown(self.on_shutdown_hook)
        rospy.logdebug("Starting Animation Server")
        server = PlayAnimationAction(rospy.get_name())
        rospy.spin()

    def on_shutdown_hook(self):
        # we got external shutdown, let's still wait a bit, since we propably want to do a shut down animation
        rospy.sleep(5)


class PlayAnimationAction(object):
    _feedback = PlayAnimationFeedback
    _result = PlayAnimationResult

    def __init__(self, name):
        self.current_joint_states = None
        self.action_name = name
        self.hcm_state = 0
        self.current_animation = None
        self.animation_cache = {}
        all_animations = find_all_animations_by_name()
        for animation_name, animation_file in all_animations.items():
            try:
                with open(animation_file) as fp:
                    self.animation_cache[animation_name] = parse(json.load(fp))
            except IOError:
                rospy.logerr("Animation '%s' could not be loaded" % animation_name)
            except ValueError:
                rospy.logerr(
                    "Animation '%s' had a ValueError. Probably there is a syntax error in the animation file. "
                    "See traceback" % animation_name)
                traceback.print_exc()

        # predefined messages for performance
        self.anim_msg = AnimationMsg()
        # AnimationMsg takes a JointTrajectory message to also be able to process trajectories. To keep this
        # functionality, we use this message type, even though we only need a single joint goal in this case.
        self.traj_msg = JointTrajectory()
        self.traj_point = JointTrajectoryPoint()

        rospy.Subscriber("joint_states", JointState, self.update_current_pose, queue_size=1)
        rospy.Subscriber("robot_state", RobotControlState, self.update_hcm_state, queue_size=1)
        self.hcm_publisher = rospy.Publisher("animation", AnimationMsg, queue_size=1)

        self._as = actionlib.SimpleActionServer(self.action_name, PlayAction,
                                                execute_cb=self.execute_cb, auto_start=False)

        self._as.start()

    def execute_cb(self, goal):
        """ This is called, when someone calls the animation action"""
        first = True
        self.current_animation = goal.animation

        # publish info to the console for the user
        rospy.loginfo("Request to play animation %s", goal.animation)

        if self.hcm_state != 0 and not goal.hcm:  # 0 means controllable
            # we cant play an animation right now
            # but we send a request, so that we may can soon
            self.send_animation_request()
            rospy.loginfo("HCM not controllable. Only sent request to make it come controllable.")
            self._as.set_aborted(text="HCM not controllable. Will now become controllable. Try again later.")
            return

        animator = self.get_animation_splines(self.current_animation)
        # start animation
        rate = rospy.Rate(500)

        while not rospy.is_shutdown() and animator:
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
            t = rospy.get_time() - animator.get_start_time()
            pose = animator.get_positions_rad(t)
            if pose is None:
                # see walking node reset

                # animation is finished
                # tell it to the hcm
                self.send_animation(False, True, goal.hcm, None, None)
                self._as.publish_feedback(PlayAnimationFeedback(percent_done=100))
                # we give a positive result
                self._as.set_succeeded(PlayAnimationResult(True))
                return

            self.send_animation(first, False, goal.hcm, pose, animator.get_torque(t))
            first = False  # we have sent the first frame, all frames after this can't be the first
            perc_done = int(((rospy.get_time() - animator.get_start_time()) / animator.get_duration()) * 100)
            perc_done = max(0, min(perc_done, 100))
            self._as.publish_feedback(PlayAnimationFeedback(percent_done=perc_done))

            try:
                # catch exception of moving backwards in time, when restarting simulator
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                rospy.logwarn("We moved backwards in time. This is probably because the simulation was reset.")
            except rospy.exceptions.ROSInterruptException:
                exit()

    def get_animation_splines(self, animation_name):
        if animation_name not in self.animation_cache:
            rospy.logerr("Animation '%s' not found" % animation_name)
            self._as.set_aborted(False, "Animation not found")
            return

        parsed_animation = self.animation_cache[animation_name]
        return SplineAnimator(parsed_animation, self.current_joint_states)

    def check_for_new_goal(self):
        if self._as.is_new_goal_available():
            next_goal = self._as.next_goal
            if not next_goal or not next_goal.get_goal():
                return
            rospy.logdebug("New goal: " + next_goal.get_goal().animation)
            if next_goal.get_goal().hcm:
                rospy.logdebug("Accepted hcm animation %s", next_goal.get_goal().animation)
                # cancel old stuff and restart
                self._as.current_goal.set_aborted()
                self._as.accept_new_goal()
            else:
                # can't run this animation now
                self._as.next_goal.set_rejected()
                # delete the next goal to make sure, that we can accept something else
                self._as.next_goal = None
                rospy.logwarn("Couldn't start non hcm animation because another one is already running.")

    def update_current_pose(self, msg):
        """Gets the current motor positions and updates the representing pose accordingly."""
        self.current_joint_states = msg

    def update_hcm_state(self, msg):
        self.hcm_state = msg.state

    def send_animation_request(self):
        self.anim_msg.request = True
        self.anim_msg.header.stamp = rospy.Time.now()
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
        self.anim_msg.header.stamp = rospy.Time.now()
        self.hcm_publisher.publish(self.anim_msg)


if __name__ == "__main__":
    rospy.logdebug("starting animation node")
    animation = AnimationNode()
