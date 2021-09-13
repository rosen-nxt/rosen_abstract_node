from rosen_abstract_node.msg import NodeState

class NodeStateHelper(object):

    valid_states = {
        NodeState.STOPPED : "STOPPED",
        NodeState.NODE_CONFIGURED : "NODE_CONFIGURED",
        NodeState.COMPONENT_CONNECTED : "COMPONENT_CONNECTED",
        NodeState.COMPONENT_RUNNING : "COMPONENT_RUNNING",
        NodeState.COMPONENT_PAUSED : "COMPONENT_PAUSED",
        NodeState.COMPONENT_DISCONNECTED : "COMPONENT_DISCONNECTED"
    }

    @classmethod
    def is_valid(cls, state):
        """ Indicates whether the passed value is a valid node state.

        Parameters
        ----------
        state : int
            A number representing a node state (rosen_abstract_node.msg.NodeState)

        Returns
        -------
        Boolean
            True if the state is a valid node state, else False.
        """
        return state in cls.valid_states

    @classmethod
    def to_string(cls, state):
        """ Converts the passed, valid node state into a string.

        Parameters
        ----------
        state : int
            A number representing a valid node state (rosen_abstract_node.msg.NodeState)

        Returns
        -------
        str
            A string representation of the node state.
        """
        return cls.valid_states[state]
