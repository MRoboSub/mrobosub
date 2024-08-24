from captain import transition_maps
from inspect import signature

errors = 0
for map_name, map in transition_maps.items():
    for outcome, state in map.items():
        if len(signature(state.__init__).parameters) != 2:
            print(f"{state}.__init__ has wrong number of parameters")
            errors += 1
        if not state.is_valid_income_type(outcome):
            print(f'{state} is mapped to by invalid outcome {outcome} in transition map "{map_name}"')
            errors += 1

if errors == 0:
    print("No errors found")
else:
    print(f"{errors} error(s) found")
