from abc import ABCMeta, abstractmethod
from threading import Condition

import actionlib
import diagnostic_updater
import rospy
from std_msgs.msg import UInt8

from rosen_abstract_node.node_transition_helper import NodeTransitionHelper
from rosen_abstract_node.abstract_node_sm import AbstractNodeSM
from rosen_abstract_node.msg import NodeStateInfo, NodeTransition, StateTransitionAction, StateTransitionFeedback, StateTransitionResult

class RosenAbstractNode(object):
    __metaclass__ = ABCMeta

    def __init__(self, node_name=None, flags=0):
        """ Initialises the abstract node with all required ROS components with the
        given node name in the given namespaces.
        Use this constructor if you want a default initialisation of all required ROS components.
    
        Parameters
        ----------
        node_name : str
            The name of the ROS node. When passing None, name is inherited from ROS configuration.
        flags: int
            Additional node control parameter. They can be used by other nodes to control this node
            in a certain way. The concrete definition of these parameters is not part of the abstract_node.
        """    
        self.ros_node_name = rospy.get_name() if node_name == None else node_name
        self.flags = flags
        self.next_trans = NodeTransition.NONE
        self.transition_successful = False
        self.transition_processed = False
        self.transition_condition = Condition()

        self.sm = AbstractNodeSM(do_init=self.do_init,
                                 do_connect=self.do_connect,
                                 do_disconnect=self.do_disconnect,
                                 do_start=self.do_start,
                                 do_pause=self.do_pause,
                                 do_resume=self.do_resume,
                                 do_stop=self.do_stop)

        self.state_transition_action_server = actionlib.SimpleActionServer(
                                            "~state_transition_action",
                                            StateTransitionAction,
                                            execute_cb = self.sm_action_cb,
                                            auto_start = False)
        self.state_transition_action_server.start()
        self.diag_updater = diagnostic_updater.Updater()
        self.diag_updater.setHardwareID(self.ros_node_name)

        self.flags_publisher = rospy.Publisher('~flags', UInt8, queue_size=1, latch=True)
        self.publish_flags()

    def sm_action_cb(self, goal):
        """ Callback used when initiating a transition via the state_transition_action.

        Parameters
        ----------
        goal : rosen_abstract_node.msg.StateTransitionGoal
            The goal of the transition.
        """
        self.transition_processed = False
        trans = goal.transition
        if NodeTransitionHelper.is_valid(trans) and trans != NodeTransition.NONE:
            rospy.loginfo("rosen_abstract_node::sm_action_cb transition: {}".format(NodeTransitionHelper.to_string(trans)))

            state_transition_feedback = StateTransitionFeedback()
            state_transition_feedback.current_state = self.sm.get_current_state()
            state_transition_feedback.transition = trans
            self.state_transition_action_server.publish_feedback(state_transition_feedback)

            self.initiate_transition(trans)

            with self.transition_condition:
                self.transition_condition.wait()
        else:
            rospy.logwarn("rosen_abstract_node::sm_action_cb received an invalid transition request: {}".format(trans))
            self.transition_successful = False

        state_transition_result = StateTransitionResult()
        state_transition_result.new_state = self.sm.get_current_state()
        if self.transition_successful:
            self.state_transition_action_server.set_succeeded(state_transition_result)
        else:
            self.state_transition_action_server.set_aborted(state_transition_result)

    def initiate_transition(self, transition):
        """ Initiate the given transition to be executed by the node in the next loop.
        If multiple transitions get passed before the loop executed one of them, 
        only the last one gets executed.

        Parameters
        ----------
        transition : int
            The transition (rosen_abstract_node.msg.NodeTransition) to be executed.
        """
        self.next_trans = transition

    def do_transition(self,sequence_count,node_state_info_publisher):
        """ Executes the desired transition, if possible.
        If no transition is set, nothing happens.

        Parameters
        --------
        sequence_count: int
            sequence_count that will be updated for the node_state_info_publisher
        
        node_state_info_publisher: diagnostic_updater.DiagnosedPublisher
            publishes the new current state
        """
        if self.next_trans == NodeTransition.NONE:
            return

        with self.transition_condition:
            self.transition_successful = False

            try:
                self.transition_successful = self.sm.do_transition(self.next_trans)
            except KeyError:
                message = "rosen_abstract_node::do_transition: Received invalid transition. Please check at caller!"
                rospy.logfatal(message)
                raise TypeError(message)

            sequence_count += 1
            state_info = NodeStateInfo()
            state_info.current_state = self.sm.get_current_state()
            state_info.header.seq = sequence_count
            state_info.header.stamp = rospy.Time.now()
            node_state_info_publisher.publish(state_info)
            self.transition_processed = True
            self.next_trans = NodeTransition.NONE
            self.transition_condition.notify()


    def get_transition_successful(self):
        """ Indicates whether the last transition has been executed successfuly.

        Returns
        -------
        Boolean
            True if transition was successful, else False.
        """
        return self.transition_successful

    def get_current_state(self):
        """ Gets the current state the node is in.

        Returns
        -------
        int
            The current state the node is in (uv_msgs.msg.NodeState).
        """
        return self.sm.get_current_state()

    def get_flags(self):
        return self.flags

    def loop(self):
        """ The main, blocking loop of the node. Should be called after initialising the object.
        Runs forever.
        """
        loop_frequency = rospy.get_param("~loop_frequency", 10.0)
        rospy.loginfo("{} is running with a frequency of {} Hz.".format(self.ros_node_name, loop_frequency))
        
        loop_rate = rospy.Rate(loop_frequency)
        
        freq_bounds = { 'min': 0.5, 'max': 2.0 }
        frequency_param = diagnostic_updater.FrequencyStatusParam(freq_bounds, 0.1, 10)
        internal_publisher = rospy.Publisher('~current_state', NodeStateInfo, queue_size=10, latch=False)
        node_state_info_publisher = diagnostic_updater.DiagnosedPublisher(internal_publisher, self.diag_updater, frequency_param, diagnostic_updater.TimeStampStatusParam())
        
        sequence_count = 0
        status_freq_count = 0
        while not rospy.is_shutdown():
            self.loop_once(sequence_count,node_state_info_publisher)

            # try to send status messages with ~1 Hz
            status_freq_count += 1
            if status_freq_count > loop_frequency:
                sequence_count += 1
                state_info = NodeStateInfo()
                state_info.current_state = self.sm.get_current_state()
                state_info.header.seq = sequence_count
                state_info.header.stamp = rospy.Time.now()
                node_state_info_publisher.publish(state_info)
                status_freq_count = 1

            self.diag_updater.update()
            loop_rate.sleep()

    def get_loop_frequency(self):
        """ Gets the frequency the node is configured to loop with.
        It defines with which frequency the node performs transitions and calls do_step().
        Is configurable by the ROS parameter ~/loop_frequency.

        Returns
        -------
        double
            The frequency the node is configured to loop with
        """
        return self.loop_frequency

    def loop_once(self,sequence_count,node_state_info_publisher):
        """ Loops the statemachine once. This includes do_transition and do_step.
        
        Parameters
        --------
        sequence_count: int
            sequence_count that will be updated for the node_state_info_publisher
        
        node_state_info_publisher: diagnostic_updater.DiagnosedPublisher
            publishes the new current state"""
        self.do_transition(sequence_count,node_state_info_publisher)
        self.do_step()

    def publish_flags(self):
        flags_msg = UInt8()
        flags_msg.data = self.flags
        self.flags_publisher.publish(flags_msg)

    @abstractmethod
    def do_init(self):
        """ Called when trying to set the node into state NODE_CONFIGURED.
        If this method returns false, the transition is not executed and the node
        stays in the state it has been in before.

        Returns
        -------
        Boolean
            True if transition is possible, else False.
        """
        pass

    @abstractmethod
    def do_stop(self):
        """ Called when trying to set the node into state STOPPED.
        If this method returns false, the transition is not executed and the node
        stays in the state it has been in before.

        Returns
        -------
        Boolean
            True if transition is possible, else False.
        """
        pass

    @abstractmethod
    def do_connect(self):
        """ Called when trying to set the node into state COMPONENT_CONNECTED.
        If this method returns false, the transition is not executed and the node
        stays in the state it has been in before.

        Returns
        -------
        Boolean
            True if transition is possible, else False.
        """
        pass

    @abstractmethod
    def do_disconnect(self):
        """ Called when trying to set the node into state COMPONENT_DISCONNECTED.
        If this method returns false, the transition is not executed and the node
        stays in the state it has been in before.

        Returns
        -------
        Boolean
            True if transition is possible, else False.
        """
        pass

    @abstractmethod
    def do_pause(self):
        """ Called when trying to set the node into state COMPONENT_PAUSED.
        If this method returns false, the transition is not executed and the node
        stays in the state it has been in before.

        Returns
        -------
        Boolean
            True if transition is possible, else False.
        """
        pass

    @abstractmethod
    def do_resume(self):
        """ Called when trying to set the node into state COMPONENT_RUNNING (from COMPONENT_PAUSED).
        If this method returns false, the transition is not executed and the node
        stays in the state it has been in before.

        Returns
        -------
        Boolean
            True if transition is possible, else False.
        """
        pass
    
    @abstractmethod
    def do_start(self):
        """ Called when trying to set the node into state COMPONENT_RUNNING or COMPONENT_PAUSED (from COMPONENT_CONNECTED).
        If this method returns false, the transition is not executed and the node
        stays in the state it has been in before.

        Returns
        -------
        (Boolean, Boolean)
            First element True if transition is possible, else False.
            If second element True, the component should be set to COMPONENT_RUNNING, else to COMPONENT_PAUSED.
        """
        pass
    
    @abstractmethod
    def do_step(self):
        """ Override this method if your code needs to be called periodically (e.g. to publish status messages).
        This method will be called in the node's main loop independent of the
        node's state. The default frequency is 10 Hz.
        """
        pass
