from umrsm import *
import graphviz
import xdot

from common import *
from prequal_front import *
from prequal_turn import *
import standard_run
import prequal_turn

from typing import Mapping, Type

from pathlib import Path

def generate_graph(name: str, transitions: Mapping[Type[Outcome], Type[State]]) -> graphviz.Digraph:
    GLOBAL_NAME = 'Global'

    dot = graphviz.Digraph(name)
    dot.attr('graph', diredgeconstraints='true')
    
    for name in {state_class.__name__ for state_class in transitions.values()}:
        dot.node(name, name)

    has_global = False
    for key in transitions.keys():
        source = getattr(key, '_classname', GLOBAL_NAME)
        # print(key._class.__name__)   
        label = key.__name__
        dest = transitions[key].__name__

        if not has_global and source == GLOBAL_NAME:
            dot.node(GLOBAL_NAME, GLOBAL_NAME)
            has_global = True 

        dot.edge(source, dest, label)

    return dot

graph = generate_graph('Standard Run', standard_run.transitions)
graph.save('/root/catkin_ws/src/mrobosub/machine.dot')

# graph = generate_graph('Prequal', prequal_turn.transitions)
# graph.save('/root/catkin_ws/src/mrobosub/prequal_turn.dot')
