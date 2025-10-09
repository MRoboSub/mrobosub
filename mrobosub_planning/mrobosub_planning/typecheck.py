from mrobosub_planning.captain import transition_maps
from mrobosub_planning.umrsm import State
from inspect import signature

errors = 0
for map_name, map in transition_maps.items():
    expected_init_params = len(signature(State.__init__).parameters)
    for outcome, state in map.items():
        init_params = len(signature(state.__init__).parameters)
        if init_params != expected_init_params:
            print(
                f"{state}.__init__ has wrong number of parameters (expected {expected_init_params}, has {init_params})"
            )
            errors += 1
        if not state.is_valid_income_type(outcome):
            print(
                f'{state} is mapped to by invalid outcome {outcome} in transition map "{map_name}"'
            )
            errors += 1
        if state._num_unexpected_params != 0:
            print(
                f"{state} contains {state._num_unexpected_params} unexpected parameters"
            )
            errors += state._num_unexpected_params

if errors == 0:
    print("No errors found")
else:
    print(f"{errors} error(s) found")
