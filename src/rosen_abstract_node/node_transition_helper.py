from rosen_abstract_node.msg import NodeTransition

class NodeTransitionHelper(object):

    valid_transitions = {
        NodeTransition.NONE : "NONE",
        NodeTransition.INIT : "INIT",
        NodeTransition.CONNECT : "CONNECT",
        NodeTransition.DISCONNECT : "DISCONNECT",
        NodeTransition.START : "START",
        NodeTransition.PAUSE : "PAUSE",
        NodeTransition.RESUME : "RESUME",
        NodeTransition.STOP : "STOP"
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
