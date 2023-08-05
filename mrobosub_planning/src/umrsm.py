"""Contains the state machine implementation. You probably shouldn't read this unless you want to deal with wierd
Python metaprogramming."""

from __future__ import annotations
from abc import ABC, ABCMeta, abstractmethod
from dataclasses import make_dataclass, field
from typing import Mapping, Type, Final, cast
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest
from periodic_io import PIO

from copy import copy

STATE_TOPIC = 'captain/current_state'
SOFT_STOP_SERVICE = 'captain/soft_stop'

__all__ = ('Outcome', 'State', 'TimedState', 'ForwardAndWait', 'TurnToYaw', 'StateMachine', 'Param')

Param = Final
""" Param generic type used for annotations. The annotations themselves do nothing at runtime. 

parameters are loaded as class variables into State subclasses and should be accessed using self.{name}

they are loaded in the following order, where later values with the same name will overwrite earlier ones.

loaded into all states:
1. ~globals/defaults
2. ~globals/{machine}

loaded into all states in a particular module:
3. ~{module}/mod/defaults
4. ~{module}/mod/{machine}
the module __main__ [i.e. the module of the file which gets launched rather than imported] 
    uses the module name 'globals'. 

loaded into the particular state:
5. ~{module}/{state}/defaults
6. ~{module}/{state}/{machine}
the state name is converted to all lowercase without spaces or underscores. 

the purpose of loading parameters both for the defaults and {machine} namespaces is so that states shared
    between multiple state machines can override parameters when used in a particular machine if necessary.
    you should generally prefer to use 'defaults' unless there is a good reason not to.

the purpose of loading parameters from the '{module}/mod' namespace is for parameters which are needed across 
    all states in a particular module, but not necessarily all states in a machine. it is often the case 
    that states for a machine are divided across different files for logical organization, so it may also 
    be the case that these states have shared state

the purpose of loading parameters for the 'global' namespace is for parameters shared across all states.
    it is rarely the case that such parameters exist, but the namespace can also be used for defaults
    which are overrided for particular states [e.g. a default threshold which is only overridden when necessary]
"""

class Outcome(ABC):
    """Abstract base classes for state outcomes.

    Outcomes are used to decide which state to run next. This decision is based only on the type of outcome, and not on
    any data that the outcome contains. In the following example, instances of outcomeA map to the same State, while
    outcomeB will map to a different state.

    Example:
        >>> OutcomeA = Outcome.make('OutcomeA', offset=int)
        >>> OutcomeB = Outcome.make('OutcomeB')
        >>> outcome_a_1 = OutcomeA(offset=10)
        >>> outcome_a_2 = OutcomeA(offset=-5)
        >>> outcome_b = OutcomeB()
    """

    @classmethod
    def make(cls, name: str, **kwargs: Mapping[str, Type]) -> Type[Outcome]:
        """Returns a dataclass type (not an instance of a class, but a new class type) which is a subclass of Outcome
        and has the fields (with types) specified by kwargs

        Args:
            name: The name of the outcome; should always be the same as the variable it's assigned to.

        Example:
            >>> ReachedTargetOutcome = Outcome.make('ReachedTargetOutcome', duration=int)
            >>> current_outcome = ReachedTargetOutcome(duration=10)
            >>> current_outcome
            'ReachedTargetOutcome(duration=10)'
            >>> current_outcome = ReachedTargetOutcome()
            'ReachedTargetOutcome(duration=0)'
        """
        fields = [(key, val, field()) for key, val in kwargs.items()]
        dataclass = make_dataclass(name, fields, bases=(cls, ))
        return cast(Type[Outcome], dataclass)  # purely for better type hinting, no actual effect


class StateMeta(ABCMeta):
    def __new__(mcls: type[StateMeta], name: str, bases: tuple[type, ...], namespace: dict[str, Any], **kwargs: Any) -> StateMeta:
        try:
            for var, type_ in namespace["__annotations__"].items():
                if type_ is Outcome:
                    namespace[var] = Outcome.make(name + var)
        except AttributeError:
            pass
        return super().__new__(name, bases, namespace, **kwargs)


class State(ABC):
    """States contain logic that will be executed by the StateMachine.

        Each State also contains class variables for each parameter on the parameter server, which
            can be accessed using self. Data that should be shared between calls of handle should be 
            set as an instance variable.
    """

    def __init__(self, prev_outcome: Outcome):
        self.initialize(prev_outcome)

    @abstractmethod
    def initialize(self, prev_outcome: Outcome) -> None:
        """Runs when the state is initialized.

        This is run whenever the same state isn't run consecutively, for example in the following flow initialize is
        run for states prefixed with *:

            *Start -> *StateA -> StateA -> *StateB -> StateB -> *StateA -> *End

        prev_outcome is the Outcome object returned by the previous state, which may be used for initializing
                any instance variables, if you would like.
        """
        pass

    @abstractmethod
    def handle(self) -> Outcome:
        """Contains the logic to be run for a particular state.

        Is called repeatedly for each iteration of the state, including the first one.
        """
        pass


class TimedState(State):
    """ base class for States which can be timed out. 

        expects an outcome called TimedOut and parameter named timeout.
        override handle_once_timedout iff cleanup is needed after timeout
    """
    def initialize(self, prev_outcome: Outcome) -> None:
        self.start_time = rospy.get_time()
    
    def handle(self) -> Outcome:
        if rospy.get_time() - self.start_time >= self.timeout:
            self.handle_once_timedout()
            return self.TimedOut()
        else:
            return self.handle_if_not_timedout()

    @abstractmethod
    def handle_if_not_timedout(self) -> Outcome:
        pass

    def handle_once_timedout(self) -> None:
        pass

class ForwardAndWait(State):
    """
    Must specify the following outcomes:
    Unreached
    Reached

    Must specify the following parameters:
    target_surge_time: float
    wait_time: float
    surge_speed: float
    """
    def initialize(self, prev_outcome: Outcome) -> None:
        self.start_time = rospy.get_time()
        self.waiting = False

    def handle(self) -> Outcome:
        if not self.waiting:
            PIO.set_target_twist_surge(self.surge_speed)
            PIO.set_target_pose_heave(self.target_heave)

            if rospy.get_time() - self.start_time >= self.target_surge_time:
                PIO.set_target_twist_surge(0)
                self.waiting = True
                self.start_time = rospy.get_time()
        else:
            PIO.set_target_twist_surge(0)
        
            if rospy.get_time() - self.start_time >= self.wait_time:
                return self.Reached()
        
        return self.Unreached()
        
class DoubleTimedState(State):
    """
    Must specify the following outcomes:
    Unreached
    Reached

    Must specify the following parameters:
    phase_one_time: float
    phase_two_time: float
    """
    def initialize(self, prev_outcome: Outcome) -> None:
        self.start_time = rospy.get_time()
        self.timed_out_first = False

    def handle(self) -> Outcome:
        if not self.timed_out_first:
            outcome = self.handle_first_phase()
            if rospy.get_time() - self.start_time >= self.phase_one_time:
                self.timed_out_first = True
                self.start_time = rospy.get_time()
        else:
            outcome = self.handle_second_phase()
            if rospy.get_time() - self.start_time >= self.phase_two_time:
                outcome = self.handle_once_timedout()

        return outcome
        

class TurnToYaw(TimedState):
    """
    Must specify following outcomes:
    Unreached
    Reached
    TimedOut

    Must specify following parameters:
    target_yaw: float
    yaw_threshold: float
    settle_time: float
    timeout: float
    """
    def handle_if_not_timedout(self) -> Outcome:
        PIO.set_target_pose_yaw(self.target_yaw)
        
        if not PIO.is_yaw_within_threshold(self.yaw_threshold):
            self.timer = rospy.get_time()

        if rospy.get_time() - self.timer >= self.settle_time:
            return self.Reached()

        return self.Unreached()

class StateMachine:
    """ The main interface for running a system. """
    def __init__(self, name: str, transitions: Mapping[Type[Outcome], Type[State]], StartState: Type[State], StopState: Type[State]):
        """ Creates a new state machine.

            You should call this code once for any particular run. 
            name: a string representing the name of the machine (used for overriding default parameters)
            transitions: a dictionary from Outcomes to States. Each of these should be the class itself, rather
                    than an instance of the class
            StartState: the class of the state to begin with
            StopState: the class of the state to end with. when this state is reach, its handle will be 
                called once, then the run method will return.
        """
        self.name = name
        self.StartState = StartState
        self.transitions = transitions
        self.StopState = StopState

        self._soft_stop_srv = rospy.Service(SOFT_STOP_SERVICE, Trigger, self.soft_stop)
        self.stop_signal_recvd = False

        self._load_params(State, 'globals', 'defaults')
        self._load_params(State, 'globals', self.name)
        states = set(transitions.values()).union({self.StartState})
        for state in states:
            self._load_params(state, state.__module__, 'mod', 'defaults')
            self._load_params(state, state.__module__, 'mod', self.name)
            self._load_params(state, state.__module__, state.__name__.lower(), 'defaults')
            self._load_params(state, state.__module__, state.__name__.lower(), self.name)

    @staticmethod
    def _load_params(state: Type[State], module: str, state_name, machine_name: str = '') -> None:
        """Set all parameters in the module/state_name/machine_name namespace of the ROS parameter services as class
        variables in state."""
        if module == '__main__':
            module = 'globals'
        namespace = f'~{module}/{state_name}/{machine_name}' if machine_name else f'~{module}/{state_name}'
        print(f'loading namespace {namespace}')
        for key, value in rospy.get_param(namespace, {}).items():
            print(f'\t{key}: {value}')
            setattr(state, key, value)  # read-only constants

    def soft_stop(self, data: TriggerRequest) -> Tuple[bool, string]:
        self.stop_signal_recvd = True
        return True, type(self.current_state).__qualname__

    def run(self) -> Outcome:
        """ Performs a run, beginning with the StartState and ending when it reaches StopState.
            
            Returns the Outcome from calling handle() on StopState.
        """
        rate = rospy.Rate(50)
        publisher = rospy.Publisher(STATE_TOPIC, String, queue_size=1)
        self.current_state = self.StartState(None)
        while type(self.current_state) != self.StopState:
            publisher.publish(type(self.current_state).__qualname__)
            outcome = self.current_state.handle()
            outcome_type = type(outcome) if isinstance(outcome, Outcome) else outcome
            outcome_name = outcome_type.__qualname__
            if self.stop_signal_recvd:
                NextState = self.StopState
                outcome_name = '!! Abort !!'
            else:
                NextState = self.transitions[outcome_type]
            rospy.logdebug(f'{type(self.current_state).__qualname__} -> {NextState.__qualname__}')
            if type(self.current_state) != NextState:
                rospy.loginfo(f'transition {type(self.current_state).__qualname__} --[{outcome_name}]--> {NextState.__qualname__}')
                self.current_state = NextState(outcome)
            rate.sleep()
        publisher.publish(type(self.current_state).__qualname__)
        return self.current_state.handle()  # handle stop state
