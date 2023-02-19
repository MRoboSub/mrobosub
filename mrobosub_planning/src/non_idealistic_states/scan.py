from state_machine import Outcome, State, Param

class Scan(State):
    NotFound = Outcome.make("NotFound")
    TimedOut = Outcome.make("TimedOut")
    FoundPathMarker = Outcome.make("FoundPathMarker")
