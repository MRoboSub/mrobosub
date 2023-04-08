#!/usr/bin/env python

import rospy
from typing import NewType, TypeVar, Final, Sequence, Any, Generic, Callable, get_type_hints
from inspect import getfullargspec
from std_srvs.srv import Empty, EmptyResponse

Param = Final

class MissingParameterError(Exception):
    def __init__(self, name):
        super().__init__(f'missing parameter {name}')

class Node:
    def __init__(self, name: str):
        rospy.init_node(name, anonymous=False)
        rospy.loginfo(f'starting node {name}...')

        def set_params(req=None):
            rospy.loginfo('(re)loading parameters...')
            params = rospy.get_param('~', {})
            for key, val in params.items():
                setattr(self, key, val)
                rospy.logdebug(f'loaded parameter {key} := {val}')
            return EmptyResponse

        set_params()
        for name, typ in get_type_hints(self.__class__).items():
            if typ is Param and getattr(self, name) is None:
                raise MissingParameterError(name)

        self.param_reset_srv = rospy.Service('~/reset_params', Empty, set_params)

        rospy.on_shutdown(self.cleanup)

    def run(self):
        pass

    def cleanup(self):
        pass


class ControlLoopNode(Node):
    def __init__(self, name: str):
        super().__init__(name)

        self.rate = rospy.Rate(self.iteration_rate)

    def run(self):
        while not rospy.is_shutdown():
            self.loop()
            self.rate.sleep()

    def loop(self):
        pass

class PipelineNode(Node):
    def __init__(self, name: str):
        super().__init__(name)

def subscriber(topic_name, MsgType=None):
    def subscriber_factory(callback):
        cb_annots = get_type_hints(callback).copy()
        print(cb_annots)
        cb_annots.pop('return', None) # remove the return type annotation if it exists
        if len(cb_annots) > 1:
            raise ValueError('callback must accept exactly one argument')
        elif len(cb_annots) is 0:
            if MsgType is None:
                raise ValueError('must either annotate callback parameter or explicitly provide MsgType')
        else: # len(cb_annots) is 1
            _, MsgType = cb_annots.popitem()

        # TODO: currently broken; does not pass self as first parameter, making it unable to be a method

        return rospy.Subscriber(topic_name, MsgType, callback)

    return subscriber_factory



# M = TypeVar('M')
# class Subscriber(Generic[M]):
#     def __init__(self, topic_name: str, callback: Callable[[M], None], MsgType=None):
#         cb_annots = get_type_hints(callback).copy()
#         cb_annots.pop('return', None) # remove the return type annotation if it exists
#         if len(cb_annots) > 1:
#             raise InvalidArgument('callback must accept exactly one argument')
#         elif len(cb_annots) is 0:
#             if MsgType is None:
#                 raise InvalidArgument('must either annotate callback parameter or explicitly provide MsgType')
#         else: # len(cb_annots) is 1
#             _, MsgType = cb_annots.popitem()

#         self.subscriber = rospy.Subscriber(topic_name, MsgType, callback)

# T = TypeVar('T')
# class SubscribedVar(Generic[T]):
#     def __init__(self, topic, cls, initial=None):
#         self.val = initial
#         rospy.Subscriber(topic, cls, lambda msg: self._set(msg))

#     def _set(self, msg: T):
#         self.val = msg
