# state machine documentation

## overview
high level planning consists of three kinds of things:
- `State` -- a stage of operation that the system can be in during any given timestep
- `Outcome` -- a result from a single timestep of the system
- `StateMachine` -- a system for deciding what the system should do in the next timestep based on the outcome from a previous timestep

like all python code, these are organized into *modules*, which also correspond to how constant *parameters* are set. like all ros nodes, the system needs to be able to *publish* and *subscribe* to topics. in addition, we recommend a system of *launch files* for startup.

this guide will review syntax, semantics, and style recommendations for creating a planning system using this state machine architecture. it is intended to be a comprehensive explanation of system usage. the examples in code blocks throughout this document can serve as a quick reference for implementors.


## states

### theory
conceptually, a state is a stage of operation that the system can be in during any given timestep. when the state in some given timestep is different from the previous timestep, we say *the system left the state*. if the system returns to the same state, we say *the system remains in the state*. while the system remains in the same state, there is a persistent *local context* which the system may make use of. note that the system may leave the state, then return at a later time. in this case, although we are in the same state, the local context resets. 

### methods
in order to define a state, the user should create a subclass of `State` in the way you normally would in python. the user is expected to override two methods: `initialize` and `handle`. 

`initialize` is responsible for establishing any necessary local context in order for the state to function by setting attributes of `self`. in doing so, it can make use of the previous state's outcome (see [state transitions]). note that `handle` **is** called immediately after `initialize` in the first iteration of a particular state. additonally, note that `initialize` is called exactly once each time a state is entered from a different state. that is, if the system transitions from state S to state T, then back to state S, `intialize` will be called exactly twice for state S and once for state T.

`handle` is responsible for the normal iteration logic of the state. `handle` can and should both read and write from the local context using attributes of `self`. `handle` must return an `Outcome` representing the result of that particular iteration. `handle` **should not perform any blocking operations**. 

you may choose to implement additional helper methods as necessary. if you choose to override the default `__init__`, you should be sure to take the previous outcome as an argument and call the `initialize` method (either directly or by calling `super().__init__()`). you may also wish to override the default `__repr__`, which is called to represent the state for logging purposes. the default implementation does not show any class attributes.

### implementation details
in defining a subclass of `State`, the user is defining a new type. an instance of this type is constructed each time a state is entered. in each iteration of the `StateMachine`, if the returned `Outcome` maps to the current `State`, then no new object is constructed and the `handle` method is called. otherwise, a new object is constructed and its `__init__()` is called with the argument of the previous `Outcome` (the default implementation of `__init__()` will then call `intialize()` with the same argument). the curious reader may ask why the authors chose to force the user to override a separately called `intialize()` method rather than simply `__init__`. the reason is because python

### example
```python
import rospy
from state_machine import *

class TimedState(State):
    timeout : Param[int]

    def intialize(self, prev_outcome: Outcome) -> None:
        self.start_time = rospy.get_time()
        self.data = prev_outcome.data

    def handle(self) -> Outcome:
        if (rospy.get_time() - self.start_time > self.timeout):
            return TimedOut()
        else:
            self.data *= 2
            return DoubledData(self.data)
```
in this example, `TimedOut` and `DoubledData` are subclasses of `Outcome` defined elsewhere and `timeout` is a constant from the ros parameter server.



## outcomes

### theory
an outcome is the result of a single timestep of the system. the system then uses the *type* of this outcome to determine the next state to transition to. *instances* of outcomes may also contain certain data (as defined by its type) to aid in the intialization logic for the next state. if the system remains in the same state, the outcome is discarded, since any relevant data associated with it should already be a part of the local context of the state.

### syntax and semantics
`Outcome` types are defined using the `Outcome.make()` class method. The method takes the desired class name as a string and an arbitrary sequence of keyword arguments used to define the class's attributes and their default values. The returned object is a *class*. this class has the passed-in name, is a subclass of `Outcome`, and has the following provided methods, corresponding to those created by python's `make_dataclass()`:
- `__init__()` -- initializer for the class. arguments provided positionally correspond to the order in which the attributes are specified in the call for `Outcome.make()`. arguments may also be provided as keyword arguments using the same names as the keywords in `Outcome.make()`. any attributes which are not specified in the arguments to the initializer are given the default values specified as the values in `Outcome.make()`. attributes which are passed to `__init__` should have the same type as the corresponding default value specified in `Outcome.make()`.
- `__repr__()` -- represents the outcome intstance as a string, using its name and its attributes formatted like keyword arguments
- `__eq__()` -- compares two instances of the same `Outcome` type through pairwise comparison of the fields

note that the attributes in instances of `Outcome` types are not frozen (i.e. mutable). as a consequence, `Outcome` instances are not hashable.

exactly one new `Outcome` instance should be constructed and returned in each timestep of the system in the `handle` method of the system's current `State`.


### style and organization
names of `Outcome` subclasses should be phrased as results. often, this takes the form of a past-tense verb. note that, although the types are returned from a function, they are, in fact, types. thus, you should use the python convention of PascalCase rather than snake_case. additionally, although it is technically well-defined, it would be quite silly to make the name passed as a string to `Outcome.make` different from the name of the object its return value is assigned to. a possible exception to this guideline is prefixing the string name with the name of the state it belongs to, but not the object name.


when constructing an instance of an outcome, you should prefer the keyword-argument style, especially when there are multiple attributes. you should only use the positional style if the data is clearly associated with the name of the outcome and has an intuitive ordering. `Outcome` instances can generally be constructed in the return position, but if they must be named, be sure to use snake_case.


`Outcome` types are typically closely associated with a particular state. for example, you likely have a series of `Outcome` types which represent the possible exit conditions of any given `State`. as such, `Outcome` types should generally be defined within the `State` subclass they are associated with. doing so will make the association clear and avoid name collisions with similar but unrelated `Outcome` types associated with other `State` types. to that point, recall that every instance of a given `Outcome` type will transition to the same `State`. therefore, even if different states can all have corresponding results, they should generally be different classes. for example, many states will be able to time out, and therefore may have a `TimedOut` type. however, since one may wish to construct a machine in which different states timing out transitions to different states, they should not all be the same type.


## state machines



## modules


## periodic io

## ros parameter server

## launch files
