from rosen_abstract_node.state_machine import StateMachine
from rosen_abstract_node.msg import NodeState, NodeTransition


EMPTY_CB = lambda: True
EMPTY_START_CB = lambda: True, True

class AbstractNodeSM(object):
    def __init__(self, do_init=EMPTY_CB,
                       do_connect=EMPTY_CB,
                       do_disconnect=EMPTY_CB,
                       do_start=EMPTY_START_CB,
                       do_pause=EMPTY_CB,
                       do_resume=EMPTY_CB,
                       do_stop=EMPTY_CB):
        """
        Parameters
        ----------
        do_init : optional, function
            Optional function, which is during the INIT transition called.
        do_connect : optional, function
            Optional function, which is during the CONNECT transition called.
        do_disconnect : optional, function
            Optional function, which is during the DISCONNECT transition called.
        do_start : optional, function
            Optional function, which is during the START transition called.
        do_pause : optional, function
            Optional function, which is during the PAUSE transition called.
        do_resume : optional, function
            Optional function, which is during the RESUME transition called.
        do_stop : optional, function
            Optional function, which is during the STOP transition called.
        """    
        self._sm = StateMachine(NodeState.STOPPED)
        self._do_callbacks = {NodeTransition.INIT: do_init,
                              NodeTransition.CONNECT: do_connect,
                              NodeTransition.DISCONNECT: do_disconnect,
                              NodeTransition.PAUSE: do_pause,
                              NodeTransition.RESUME: do_resume,
                              NodeTransition.STOP: do_stop}
        self._do_start = do_start

        self._init_sm()

    def is_in_node_state(self, state):
        """ Check if the node is in a certain state.

        Parameters
        --------
        state: NodeState
            The node state, to be checked.
        
        Returns
        -------
        Boolean
            Whether or not the node is in the given state.
        """
        return self._sm.current_state == state

    def do_transition(self, next_trans):
        """ Perform a transition.

        Parameters
        --------
        state: NodeTransition
            The transition, which should be performed.
        
        Returns
        -------
        Boolean
            Whether or not the transition was successful.
        """
        # The start transition is an edge case, because it can result in different states, running or paused,
        # which depends on running parameter. So the start transition cannot be added to the state machine
        # and has to be implemented outside of it.
        if next_trans == NodeTransition.START:
            return self._sm_start()
        else:
            return self._sm.do_transition(next_trans)

    def get_current_state(self):
        """ Gets the current state of the node.

        Returns
        -------
        Boolean
            The current state of the node.
        """
        return self._sm.current_state

    def _init_sm(self):
        self._sm.add_transition(NodeTransition.INIT,
                                [NodeState.STOPPED],
                                NodeState.NODE_CONFIGURED,
                                self._do_callbacks[NodeTransition.INIT])
        self._sm.add_transition(NodeTransition.CONNECT,
                                [NodeState.NODE_CONFIGURED, NodeState.COMPONENT_DISCONNECTED],
                                NodeState.COMPONENT_CONNECTED,
                                self._do_callbacks[NodeTransition.CONNECT])
        self._sm.add_transition(NodeTransition.DISCONNECT,
                                [NodeState.COMPONENT_PAUSED, NodeState.COMPONENT_RUNNING, NodeState.COMPONENT_CONNECTED],
                                NodeState.COMPONENT_DISCONNECTED,
                                self._do_callbacks[NodeTransition.DISCONNECT])
        self._sm.add_transition(NodeTransition.PAUSE,
                                [NodeState.COMPONENT_RUNNING],
                                NodeState.COMPONENT_PAUSED,
                                self._do_callbacks[NodeTransition.PAUSE])
        self._sm.add_transition(NodeTransition.RESUME,
                                [NodeState.COMPONENT_PAUSED],
                                NodeState.COMPONENT_RUNNING,
                                self._do_callbacks[NodeTransition.RESUME])
        self._sm.add_transition(NodeTransition.STOP,
                                [NodeState.NODE_CONFIGURED, NodeState.COMPONENT_DISCONNECTED],
                                NodeState.STOPPED,
                                self._do_callbacks[NodeTransition.STOP])

    def _sm_start(self):
        if (self._sm.current_state == NodeState.COMPONENT_CONNECTED):
            successful, running = self._do_start()
            if successful:
                if running:
                    self._sm.current_state = NodeState.COMPONENT_RUNNING 
                else:
                    self._sm.current_state = NodeState.COMPONENT_PAUSED
                return True
        return False
