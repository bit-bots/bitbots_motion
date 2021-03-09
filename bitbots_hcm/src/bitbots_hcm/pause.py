#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
from humanoid_league_msgs.msg import Audio
from std_msgs.msg import Bool

from bitbots_msgs.srv import ManualPenalize
from humanoid_league_msgs.msg import GameState
from humanoid_league_speaker.speaker import speak


class Pause(object):
    """
    Always go in and out of pause by manual penalty.
    Go in and out of pause by game controller, if manual penalty is not active.
    """

    def __init__(self):
        log_level = rospy.DEBUG if rospy.get_param("/debug_active", False) else rospy.INFO
        rospy.init_node('bitbots_pause', log_level=log_level, anonymous=False)

        self.penalty_manual = False
        self.game_controller_penalty = False
        self.pause = False

        self.manual_penalize_service = rospy.Service("manual_penalize", ManualPenalize, self.manual_update)
        self.pause_publisher = rospy.Publisher("pause", Bool, queue_size=10, latch=True)
        self.speak_publisher = rospy.Publisher("speak", Audio, queue_size=10)

        self.talking = rospy.get_param("/pause/talking", True)

    def manual_update(self, req):
        if req.penalize == 0:
            # off
            self.penalty_manual = False
        elif req.penalize == 1:
            # on
            self.penalty_manual = True
        elif req.penalize == 2:
            # switch
            self.penalty_manual = not self.penalty_manual
        else:
            rospy.logerr("Manual penalize call with unspecified request")
        self.set_pause(self.penalty_manual)
        return True

    def set_pause(self, state):
        self.pause = state
        if state:
            text = "Pause!"
        else:
            text = "Unpause!"
        rospy.logwarn(text)
        speak(text, self.speak_publisher, speaking_active=self.talking, priority=90)
        self.pause_publisher.publish(Bool(state))


if __name__ == "__main__":
    pause = Pause()
    rospy.spin()
