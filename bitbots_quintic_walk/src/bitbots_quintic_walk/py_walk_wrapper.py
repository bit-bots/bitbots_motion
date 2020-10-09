from io import BytesIO

import rospy
from std_msgs.msg import Int64

from bitbots_quintic_walk.py_quintic_walk import PyWalkWrapper, init_ros, spin_once
from bitbots_msgs.msg import JointCommand, FootPressure
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String


class PyWalk(object):
    def __init__(self, namespace=""):
        # make namespace end with a /
        if namespace != "" and namespace[-1] != '/':
            namespace = namespace + "/"
        init_ros(namespace)
        self.py_walk_wrapper = PyWalkWrapper(namespace)

    def spin_ros(self):
        spin_once()

    def _to_cpp(self, msg):
        """Return a serialized string from a ROS message

        Parameters
        ----------
        - msg: a ROS message instance.
        """
        buf = BytesIO()
        msg.serialize(buf)
        value = buf.getvalue()
        return value

    def _from_cpp(self, str_msg, cls):
        """Return a ROS message from a serialized string

        Parameters
        ----------
        - str_msg: str, serialized message
        - cls: ROS message class, e.g. sensor_msgs.msg.LaserScan.
        """
        msg = cls()
        result = msg.deserialize(str_msg)
        return result

    def reset(self):
        self.py_walk_wrapper.reset()

    def step(self, dt: float, cmdvel_msg: Twist, imu_msg, jointstate_msg, pressure_left, pressure_right):
        if dt == 0.0:
            # preventing wierd spline interpolation errors on edge case
            dt = 0.001
        stepi = self.py_walk_wrapper.step(
            dt,
            self._to_cpp(cmdvel_msg),
            self._to_cpp(imu_msg),
            self._to_cpp(jointstate_msg),
            self._to_cpp(pressure_left),
            self._to_cpp(pressure_right))

        result = self._from_cpp(
            stepi,
            JointCommand
        )

        return result

    def get_left_foot_pose(self):
        foot_pose = self.py_walk_wrapper.get_left_foot_pose()
        result = self._from_cpp(foot_pose, Pose)
        return result

    def set_engine_dyn_reconf(self, param_dict):
        self.py_walk_wrapper.set_engine_dyn_reconf(param_dict)

    def set_node_dyn_reconf(self, param_dict):
        self.py_walk_wrapper.set_node_dyn_reconf(param_dict)

    def get_phase(self):
        return self.py_walk_wrapper.get_phase()

    def get_freq(self):
        return self.py_walk_wrapper.get_freq()
