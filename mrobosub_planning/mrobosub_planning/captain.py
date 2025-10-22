from importlib import import_module
from mrobosub_planning.umrsm import StateMachine, State, TransitionMap, Outcome
import mrobosub_planning.common_states as common_states
import mrobosub_planning.standard_run as standard_run

from mrobosub_lib import Node

import rclpy
from rclpy.executors import SingleThreadedExecutor
import threading
import sys
from mrobosub_planning.io_interface import Interface
import traceback


# maybe change this to something hacky like getting .transitions from the machine name module?
transition_maps: dict[str, TransitionMap] = {
    "standard": standard_run.transitions,
    # "heave_test": heave_test.transitions,
}


def state_class_from_str(full_state: str, transitions: TransitionMap) -> type[State]:
    """
    Find the associated class object from a given state name.

    Arguments:
        full_state (str): The state passed into roslaunch. Could be in the format `state` or `module.state`.
        transitions (TransitionMap): The transition map from the associated machine name.

    Returns:
        The class object for the state or a ValueError if there is an error finding the state.
    """

    def outcome_to_state_str(outcome: type[Outcome]) -> str:
        return outcome.__qualname__.split(".")[0]

    def outcome_to_state(outcome: type[Outcome]) -> type[State]:
        module = import_module(outcome.__module__)
        return getattr(module, outcome_to_state_str(outcome))

    if full_state.count(".") > 1:
        raise ValueError(f"{full_state} should have at most one '.'")

    state = full_state.split(".")[-1]
    found_outcomes = [
        outcome
        for outcome in transitions.keys()
        if outcome_to_state_str(outcome) == state
    ]
    unique_found_states = set(outcome_to_state(outcome) for outcome in found_outcomes)

    if "." in full_state:
        unique_found_states = set(
            state
            for state in unique_found_states
            if state.__module__.removeprefix("mrobosub_planning.")
            == full_state.split(".")[0]
        )

    if len(unique_found_states) != 1:
        raise ValueError(
            f"{full_state=} does not uniquely describe a state. {unique_found_states=}"
        )

    found_state = unique_found_states.pop()
    return found_state


class Captain(Node):
    def __init__(self, machine_name: str, full_state: str) -> None:
        super().__init__("captain")
        self.machine_name = machine_name
        self.starting_state = state_class_from_str(
            full_state, transition_maps[machine_name]
        )

    def run(self) -> None:
        io = Interface(self)
        machine = StateMachine(
            self.machine_name,
            transition_maps[self.machine_name],
            self.starting_state,
            common_states.Stop,
            io,
        )

        try:
            machine.run()
        except Exception:
            self.get_logger().info(f"{traceback.format_exc()}")
            self.tick = 0
            self.timer = self.create_timer(0.1, self.reset_node)
            while(self.tick < 20):
                pass
            self.timer.cancel()

    def reset_node(self) -> None:
        self.tick += 1
        io.reset_target_twist()

def main() -> None:
    rclpy.init()

    # Syntax `ros2 launch mrobosub_planning captain.launch machine:=<machine> state:=<state|module.state>`
    machine_name = sys.argv[1]
    full_state = sys.argv[2]

    captain_node = Captain(machine_name, full_state)

    executor = SingleThreadedExecutor()
    executor.add_node(captain_node)
    t = threading.Thread(target=executor.spin, daemon=False)
    t.start()

    captain_node.run()

    executor.shutdown()
    t.join()


if __name__ == "__main__":
    main()
