from state_machine import Outcome, State, Param

class AlignGate(State):
    Unaligned = Outcome.make('Unaligned')
    FoundPathMarker = Outcome.make('FoundPathMarker')
    TimedOut = Outcome.make('TimedOut')
