#!/usr/bin/env python3

from __future__ import print_function
import sys

import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from bitbots_msgs.msg import DynUpGoal, DynUpAction, DynUpFeedback

showing_feedback = False

if __name__ == "__main__":
    directions = ['front', 'front_only', 'back', 'back_only', 'squat', 'rise',
                  'descend']
    if len(sys.argv) != 2 or not sys.argv[1] in directions:
        print('Use ' + str(directions) + ' as parameters!')
        sys.exit(1)

    print("[..] Initializing node", end='')
    rospy.init_node('dynup_dummy_client', anonymous=True)
    print("\r[OK] Initializing node")


    def done_cb(state, result):
        print('Action completed: ', end='')
        if state == GoalStatus.PENDING:
            print('Pending')
        elif state == GoalStatus.ACTIVE:
            print('Active')
        elif state == GoalStatus.PREEMPTED:
            print('Preempted')
        elif state == GoalStatus.SUCCEEDED:
            print('Succeeded')
        elif state == GoalStatus.ABORTED:
            print('Aborted')
        elif state == GoalStatus.REJECTED:
            print('Rejected')
        elif state == GoalStatus.PREEMPTING:
            print('Preempting')
        elif state == GoalStatus.RECALLING:
            print('Recalling')
        elif state == GoalStatus.RECALLED:
            print('Recalled')
        elif state == GoalStatus.LOST:
            print('Lost')
        else:
            print('Unknown state', state)
        print(str(result))


    def active_cb():
        print("Server accepted action")


    def feedback_cb(feedback):
        if len(sys.argv) > 1 and '--feedback' in sys.argv:
            print('Feedback')
            print(feedback)
            print()


    print('[..] Connecting to action server \'dynup\'', end='')
    sys.stdout.flush()
    client = actionlib.SimpleActionClient('dynup', DynUpAction)
    if not client.wait_for_server():
        exit(1)
    print('\r[OK] Connecting to action server \'dynup\'')
    print()

    goal = DynUpGoal()
    goal.direction = sys.argv[1]

    client.send_goal(goal)
    client.done_cb = done_cb
    client.feedback_cb = feedback_cb
    client.active_cb = active_cb
    print("Sent new goal. Waiting for result")
    client.wait_for_result()
