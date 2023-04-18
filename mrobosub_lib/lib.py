#!/usr/bin/env python

import rospy
from typing import NewType, TypeVar, Final, Sequence, Any, Generic

Param = Final

class Node:
    def __init__(self, name):
        rospy.init_node(name, anonymous=False)
        rospy.loginfo(f'starting node {name}')

        params = rospy.get_param(name, {})
        for key in params:
            setattr(self, key, params[key])

        rospy.on_shutdown(lambda: self.cleanup())

    def run(self):
        pass

    def cleanup(self):
        pass


class ControlLoopNode(Node):
    def __init__(self, name):
        super().__init__(name)

        self.rate = rospy.Rate(self.iteration_rate)

    def run(self):
        while not rospy.is_shutdown():
            self.loop()
            self.rate.sleep()

    def loop(self):
        pass



T = TypeVar('T')
class SubscribedVar(Generic[T]):
    def __init__(self, topic, cls, initial=None):
        self.val = initial
        rospy.Subscriber(topic, cls, lambda msg: self._set(msg))

    def _set(self, msg: T):
        self.val = msg
