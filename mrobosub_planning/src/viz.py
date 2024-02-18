from umrsm import *
import graphviz

import standard_run

from typing import Mapping, Type
import os

from pathlib import Path

def generate_graph(name: str, transitions: TransitionMap) -> graphviz.Digraph:
    dot = graphviz.Digraph(name)
    dot.attr('graph', diredgeconstraints='true')

    for name in {state_class.__name__ for state_class in transitions.values()}:
        dot.node(name, name)

    for outcome, state in transitions.items():
        try:
            source = outcome.__qualname__.split('.')[-2]
        except IndexError as e:
            raise SyntaxError(f'Global states are not allowed: {outcome} ({outcome.__qualname__} defined in {outcome.__module__})') from e
        label = outcome.__name__
        dest = state.__name__

        dot.edge(source, dest, label)

    return dot

graph = generate_graph('Standard Run', standard_run.transitions)
graph.save(os.getcwd() + '/machine.dot')

# graph = generate_graph('Prequal', prequal_turn.transitions)
# graph.save('/root/catkin_ws/src/mrobosub/prequal_turn.dot')
