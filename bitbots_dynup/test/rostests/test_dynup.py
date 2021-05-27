#!/usr/bin/env python3
from bitbots_test.test_case import RosNodeTestCase
import rospy
from bitbots_msgs.msg import DynUpActionGoal

class DynupTestCase(RosNodeTestCase):

    def test_no_failed_ticks(self):
        dynup_publisher = rospy.Publisher('/dynup_goal', DynUpActionGoal, queue_size=1)
        msg = DynUpActionGoal()
        msg.goal.direction = "front"
        dynup_publisher.publish(msg)

        rospy.loginfo("This is a test message")
        self.with_assertion_grace_period(self.assertRosLogs(), t=100000)


if __name__ == "__main__":
    from bitbots_test import run_rostests
    run_rostests(DynupTestCase)
