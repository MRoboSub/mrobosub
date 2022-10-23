from state_machine import *


StartOutcome = make_outcome('Start')
SubmergeOutcome = make_outcome('Submerge', at_target=False, timeout=False)
TurnOutcome = make_outcome('Turn', at_target=False, timeout=False)
StopOutcome = make_outcome('Stop', exit_with_timout=False)

SubmergeReachedDepth = make_outcome('SubmergedReachedDepth')
SubmergeReachedDepth = make_outcome('SubmergedFailedReachedDepth')
SubmergeReachedDepth = make_outcome('SubmergedReachedDepth')


class StartHandler(StateHandler):
    def iteration(self, gbl_ctx: Context):
        pass


class Submerge(State):
    outcomes = [
        Outcome('ReachedDepth'),
    ]

    def iteration(self):
        pass


class SubmergeHandler(StateHandler):
    def iteration(self, gbl_ctx: Context):
        pass


class TurnHandler(StateHandler):
    def iteration(self, gbl_ctx: Context):
        pass


class StopHandler(StateHandler):
    def iteration(self, gbl_ctx: Context):
        pass


if __name__ == '__main__':
    Start = State('Start', StartHandler)
    Submerge = State('Submerge', SubmergeHandler)
    Turn = State('Turn', TurnHandler)
    Stop = State('Stop', StopHandler)
