# -*- coding:utf-8 -*-
"""
Wait
^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

Just waits for something (i.e. that preconditions will be fullfilled)
"""
from bitbots_hcm.hcm_dsd.hcm_blackboard import HcmBlackboard

from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class Wait(AbstractActionElement):
    """
    This action waits a specified time before it pops itself
    """

    def __init__(self, blackboard, dsd, parameters=None):
        """
        :param parameters['time']: Time to wait in seconds
        """
        super().__init__(blackboard, dsd)
        self.blackboard: HcmBlackboard
        self.time = float(self.blackboard.node.get_clock().now().seconds_nanoseconds()[0]  \
                           + self.blackboard.node.get_clock().now().seconds_nanoseconds()[1]/1e9) \
                            + float(parameters['time'])

    def perform(self, reevaluate=False):
        """
        Only pop when the wait-time has elapsed
        """

        if self.time < float(self.blackboard.node.get_clock().now().seconds_nanoseconds()[0] \
                             + self.blackboard.node.get_clock().now().seconds_nanoseconds()[1]/1e9):
            self.pop()
