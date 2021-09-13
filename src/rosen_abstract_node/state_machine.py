from dataclasses import dataclass
from typing import Callable, List

from rosen_abstract_node.msg import NodeState


@dataclass
class StateMachineTransition:
    """ This dataclass contains everything relevant for the state machine transition. """

    sources: List[NodeState]
    target: NodeState
    callback: Callable[[], bool]


class StateMachine:
    def __init__(self, initial_state):
        """ This is a general state machine. The user can add nodes and transitions.
        Parameters
        ----------
        initial_state : NodeState
            The initial state of the state machine.
        """

        self.current_state = initial_state
        self._transitions = {}

    def add_transition(self, node_transition, sources, target, callback):
        """ Add a transition with the corresponding nodes and callbacks to the state machine.

        Parameters
        ----------
        node_transition : NodeTransition
            The transition (directed edge) between the nodes.
        sources : list[NodeState]
            The source nodes of the transition. It is possible to have multiple
            source nodes for a transition to a target node.
        target : NodeState
            The target node of the transition.
        callback : Callable[[], bool]
            The callback function, which will be performed, when the transition
            is called.
        """

        transition = StateMachineTransition(sources, target, callback)
        self._transitions[node_transition] = transition

    def do_transition(self, node_transition):
        """ Perform a transition from the source node to the target node and call the callback function.

        Parameters
        ----------
        transition : NodeTransition
            The transition, which should be performed.

        Returns
        -------
        Boolean
            True if transition was successful, else false.
        """

        transition = self._transitions[node_transition]
        sources = transition.sources
        if self.current_state in sources:
            success = transition.callback()
            if success:
                self.current_state = transition.target
            return success
        return False
