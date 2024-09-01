# state machine documentation

## overview
high level planning consists of three kinds of things:
- `State` -- a stage of operation that the system can be in during any given timestep
- `Outcome` -- a result from a single timestep of the system
- `StateMachine` -- a system for deciding what the system should do in the next timestep based on the outcome from a previous timestep

like all python code, these are organized into *modules*. like all ros nodes, the system needs to be able to *publish* and *subscribe* to topics. in addition, we recommend a system of *launch files* for startup.

this guide will review syntax, semantics, and style recommendations for creating a planning system using this state machine architecture. it is intended to be a comprehensive explanation of system usage. the examples in code blocks throughout this document can serve as a quick reference for implementors.


## states

### theory
conceptually, a state is a stage of operation that the system can be in during any given timestep. when the state in some given timestep is different from the previous timestep, we say *the system left the state*. if the system returns to the same state, we say *the system remains in the state*. while the system remains in the same state, there is a persistent *local context* which the system may make use of. note that the system may leave the state, then return at a later time. in this case, although we are in the same state, the local context resets. 

### methods
in order to define a state, the user should create a subclass of `State` in the way you normally would in python. the user is expected to override two methods: `__init__` and `handle`. There are also prewritten abstract subclasses of `State` that make certain behavior easier to implement. Most states should subclass `TimedState` as it adds a timeout parameter to ensure the robot never gets permanently stuck in a single state. When subclassing another state, ensure you have implemented all required functionality according to the documentation.

`__init__` is responsible for establishing any necessary local context in order for the state to function by setting attributes of `self`. in doing so, it can make use of the previous state's outcome (see [state transitions]). note that `handle` **is** called immediately after `__init__` in the first iteration of a particular state. additonally, note that `__init__` is called exactly once each time a state is entered from a different state. that is, if the system transitions from state S to state T, then back to state S, `__init__` will be called exactly twice for state S and once for state T.

`handle` is responsible for the normal iteration logic of the state. `handle` can and should both read and write from the local context using attributes of `self`. `handle` must return an `Outcome` representing the result of that particular iteration. `handle` **should not perform any blocking operations**. 

### implementation details
in defining a subclass of `State`, the user is defining a new type. an instance of this type is constructed each time a state is entered. in each iteration of the `StateMachine`, if the `handle` method returns `None`, then no new object is constructed and the `handle` method is called in the next iteration. Otherwise, a new object is constructed and its `__init__()` is called with the argument of the previous `Outcome`.

### example
```python
import rospy
from umrsm import *
from typing import NamedTuple

class TimedState(State):
    class TimedOut(NamedTuple):
        pass
    class DoubleData(NamedTuple):
        data: Any

    timeout: int

    def __init__(self, prev_outcome: Outcome) -> None:
        super().__init__(prev_outcome)
        self.start_time = rospy.get_time()
        self.data = prev_outcome.data

    def handle(self) -> Outcome:
        if (rospy.get_time() - self.start_time > self.timeout):
            return TimedOut()
        else:
            self.data *= 2
            return DoubledData(self.data)
```


## outcomes

### theory
an outcome is the result of a single timestep of the system. the system then uses the *type* of this outcome to determine the next state to transition to. *instances* of outcomes may also contain certain data (as defined by its type) to aid in the intialization logic for the next state. if the system remains in the same state, the outcome is discarded, since any relevant data associated with it should already be a part of the local context of the state.

### syntax and semantics
`Outcome` types are subclasses of `typing.NamedTuple` (this will likely be replaced by a custom `Outcome` superclass in the near future).

Note that the attributes in instances of `Outcome` types are frozen, meaning they cannot be reassigned once created (`NamedTuple`s are effectively tuples in this regard).

Each timestep of the system in the `handle` method of the system's current `State` should return either an instance of an `Outcome`, in which case the state machine transitions to the state corresponding to that outcome, or it should return `None`, in which case the state machine stays on the current state.


### style and organization
names of `Outcome` subclasses should be phrased as results. often, this takes the form of a past-tense verb. note that, although the types are returned from a function, they are, in fact, types. thus, you should use the python convention of PascalCase rather than snake_case.

`Outcome` types should always be defined within the state to which they belong. This enhances code readability and allows the state machine visualization program to function correctly. In order to have the same data returned by multiple different states, the current advice is to create a shared `NamedTuple` base class containing the requisite data, and creating a subclass for each state requiring the outcome. However, you must be careful when subclassing `NamedTuples`, as you cannot add aditional data from within the subclass as it results in unexpected and strange behavior "because Python". This is one of the primary reasons we are looking to abandon `NamedTuple` in favor of a custom `Outcome` base class.


## state machines

## modules

## periodic io

## launch files
