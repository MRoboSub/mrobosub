from state_machine import Outcome, State, Param

class AlignPathMarker(State):
    Unaligned: Outcome
    Aligned: Outcome

class AlignPathMarker(State):
    Unaligned = Outcome.make("Unaligned", )
    Aligned = Outcome.make("Aligned")
