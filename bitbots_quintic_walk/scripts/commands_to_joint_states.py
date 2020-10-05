#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from bitbots_msgs.msg import JointCommand


# script is similar to mot_goals_viz_helper but works with any robot, but only for the actual published goals
class CommandsToJointStates:
    def __init__(self):
        rospy.init_node("commands_to_joints", anonymous=False)
        self.joint_publisher = rospy.Publisher('joint_states', JointState, queue_size=10, tcp_nodelay=True)
        rospy.Subscriber("walking_motor_goals", JointCommand, self.joint_command_cb, queue_size=10, tcp_nodelay=True)
        rospy.spin()

    def joint_command_cb(self, msg: JointCommand):
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = msg.joint_names
        # hack for nao robot which has mimic joints
        if "LHipYawPitch" in joint_state.name:
            joint_state.name.insert(6, "RHipYawPitch")
        joint_state.position = msg.positions
        joint_state.velocity = msg.velocities
        self.joint_publisher.publish(joint_state)

if __name__ == '__main__':
    ctjs = CommandsToJointStates()