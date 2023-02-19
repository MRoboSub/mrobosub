from state_machine import Outcome, State, Param

class Submerge(State):
    Unreached = Outcome.make("Unreached")
    Submerged = Outcome.make("Submerged")
    TimedOut = Outcome.make("TimedOut")
