from enum import IntFlag


class NodeStateNo(IntFlag):
    STOPPED = 1
    NODE_CONFIGURED = 2
    COMPONENT_CONNECTED = 3
    COMPONENT_RUNNING = 4
    COMPONENT_PAUSED = 5
    COMPONENT_DISCONNECTED = 6


class NodeStateHelper(object):
    valid_states = {
        NodeStateNo.STOPPED : "STOPPED",
        NodeStateNo.NODE_CONFIGURED : "NODE_CONFIGURED",
        NodeStateNo.COMPONENT_CONNECTED : "COMPONENT_CONNECTED",
        NodeStateNo.COMPONENT_RUNNING : "COMPONENT_RUNNING",
        NodeStateNo.COMPONENT_PAUSED : "COMPONENT_PAUSED",
        NodeStateNo.COMPONENT_DISCONNECTED : "COMPONENT_DISCONNECTED"
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
