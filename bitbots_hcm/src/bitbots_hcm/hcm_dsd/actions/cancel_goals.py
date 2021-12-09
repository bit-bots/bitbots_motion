import actionlib
from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class CancelGoals(AbstractActionElement):

    def __init__(self, blackboard, dsd, parameters=None):
        super(CancelGoals, self).__init__(blackboard, dsd, parameters)

    def perform(self):
        self.blackboard.dynup_action_client.cancel_all_goals()
        self.blackboard.animation_action_client.cancel_all_goals()
        self.blackboard.dynamic_kick_client.cancel_all_goals()
        self.blackboard.cancel_move_base_goal()
        self.pop()

