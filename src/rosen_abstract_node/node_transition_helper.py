from enum import IntFlag


class NodeTransitionNo(IntFlag):
    NONE = 0
    INIT = 1
    CONNECT = 2
    DISCONNECT = 3
    START = 4
    PAUSE = 5
    RESUME = 6
    STOP = 7


class NodeTransitionHelper(object):
    valid_transitions = {
        NodeTransitionNo.NONE : "NONE",
        NodeTransitionNo.INIT : "INIT",
        NodeTransitionNo.CONNECT : "CONNECT",
        NodeTransitionNo.DISCONNECT : "DISCONNECT",
        NodeTransitionNo.START : "START",
        NodeTransitionNo.PAUSE : "PAUSE",
        NodeTransitionNo.RESUME : "RESUME",
        NodeTransitionNo.STOP : "STOP"
    }

    @classmethod
    def is_valid(cls, transition):
        """ Indicates whether the passed value is a valid node transition.

        Parameters
        ----------
        transition : int
            A number representing a node transition (rosen_abstract_node.msg.NodeTransition)

        Returns
        -------
        Boolean
            True if the transition is a valid node transition, else False.
        """
        return transition in cls.valid_transitions

    @classmethod
    def to_string(cls, transition):
        """ Converts the passed, valid node transition into a string.

        Parameters
        ----------
        transition : int
            A number representing a valid node transition (rosen_abstract_node.msg.NodeTransition)

        Returns
        -------
        str
            A string representation of the node transition.
        """
        return cls.valid_transitions[transition]
