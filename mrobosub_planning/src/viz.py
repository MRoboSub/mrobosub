from umrsm import *
import graphviz

from captain import transition_maps

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

def main():
    parser = argparse.ArgumentParser(description="Create a state machine visualization")
    parser.add_argument("map_name", type=str, help="The name of the transition map from captain to be visualized")
    parser.add_argument("--output", "-o", type=Path, default='machine.dot', required=False, help="The location to save the visualization ('.dot' format)")
    args = parser.parse_args()

    graph = generate_graph('Standard Run', transition_maps[args.map_name])
    graph.save(args.output)

if __name__ == "__main__":
    import argparse
    main()
