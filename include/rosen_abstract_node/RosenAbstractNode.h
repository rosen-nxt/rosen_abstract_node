#ifndef ROSEN_ABSTRACT_NODE_ABSTRACT_NODE_H
#define ROSEN_ABSTRACT_NODE_ABSTRACT_NODE_H

#include <string>
#include <mutex>
#include <condition_variable>

#include <ros/node_handle.h>
#include <actionlib/server/simple_action_server.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include "rosen_abstract_node/NodeTransitionHelper.h"
#include "rosen_abstract_node/NodeStateHelper.h"
#include "rosen_abstract_node/NodeStateInfo.h"
#include "rosen_abstract_node/StateTransitionAction.h"
#include "rosen_abstract_node/AbstractNodeSm.h"

/**
 *
 * @author Mario Hauke (mhauke@rosen-group.com)
 * @author Peter Kampmann (peter.kampmann@dfki.de)
 * @author Michael Rohn (michael.rohn@dfki.de
 * @author Martin Fritsche (martin.fritsche@dfki.de)
 * @brief This class is the template for all components within the project. The components
 * should operate according to the state machine depicted in the software development
 * handbook in section Drivers.
 *
 */

namespace rosen_abstract_node
{
    class RosenAbstractNode
    {
        public:

            /**
             * @brief Initialises the abstract node with default node name and namespace. Only the flags are required.
             *
             * @param flags  Additional node control parameter. They can be used by other nodes to
             *               control this node in a certain way. The concrete definition of these parameters
             *               is not part of the abstract_node.
             */
            explicit RosenAbstractNode(const unsigned char flags);

            /**
             * @brief Initialises the abstract node with all required ROS components with the
             *        given node name in the given namespaces.
             *        Use this constructor if you want a default initialisation of all required ROS components.
             * 
             * @param nodeName        The name of the ROS node.
             * @param nodeNamespace   The private namespace of the ROS node.
             * @param flags           Additional node control parameter. They can be used by other nodes to
             *                        control this node in a certain way. The concrete definition of these parameters
             *                        is not part of the abstract_node.
             */
            explicit RosenAbstractNode(const std::string& nodeName = ros::this_node::getName(),
                                       const std::string& nodeNamespace = "~",
                                       const unsigned char flags = 0);

            /**
             * @brief Initialises the abstract node with the passed node_handle and diagnostics
             *        and names the ROS node by the given node name.
             *        Use this constructor if you created the ROS components externally or want to use it
             *        without ROS core running.
             * 
             * @param nodeName               The name of the ROS node.
             * @param rosNodeHandlePrivate   The private node handle to be used.
             * @param rosDiagnosticUpdater   The diagnostic updater to be used for diagnostic purposes.
             * @param flags                   Additional node control parameter. They can be used by other nodes to
             *                                control this node in a certain way. The concrete definition of these
             *                                parameters is not part of the abstract_node.
             */
            RosenAbstractNode(const std::string& nodeName,
                              const std::shared_ptr<ros::NodeHandle>& rosNodeHandlePrivate,
                              const std::shared_ptr<diagnostic_updater::Updater>& rosDiagnosticUpdater,
                              const unsigned char flags = 0);

            virtual ~RosenAbstractNode() = default;

            /**
             * @brief The main, blocking loop of the node. Should be called after initialising the object.
             *        Runs forever.
             */
            void loop();

            /** 
             * @brief Initiate the given transition to be executed by the node in the next loop.
             *        If multiple transitions get passed before the loop executed one of them, 
             *        only the last one gets executed.
             * 
             * @param transition The transition to be executed.
             */
            void initiateTransition(NodeTransitionNo transition);

            /**
             * @brief Initiate the given transition to be executed by the node in the next loop.
             *        If multiple transitions get passed before the loop executed one of them,
             *        only the last one gets executed.
             *
             * @param transition The transition to be executed.
             */
            void initiateTransition(unsigned char transition);

            /**
             * @brief Gets the current state the node is in.
             * 
             * @return The current state the node is in.
             */
            unsigned int getCurrentState() const;

            /**
             * @brief Returns the configured flags.
             *
             * @return The flags.
             */
            unsigned char getFlags() const;

        private:

            unsigned char flags;

            /**
             * @brief Callback used when initiating a transition via the state_transition_action.
             * 
             * @param goal The goal of the transition.
             */
            void smActionCb(const StateTransitionGoalConstPtr& goal);

            const std::string rosNodeName;

            NodeTransitionNo nextTrans;

            bool transitionProcessed = false;
            bool transitionSuccessful = false;
            std::mutex transitionMutex;
            std::condition_variable transitionConditionVariable;

            std::shared_ptr<actionlib::SimpleActionServer<StateTransitionAction>> stateTransitionActionServer;

            /**
             * @brief Update the nodeStateInfo and publish it via ROS.
             *
             * @param currentStatePublisher The publisher of the current state.
             * @param now                   The current time.
             * @param nodeStateInfo         This is the node state, which will updated and published.
             */
            void updateAndPublishNodeStateInfo(const ros::Publisher& currentStatePublisher, const ros::Time now, NodeStateInfo& nodeStateInfo);

            std::shared_ptr<diagnostic_updater::Updater> diagUpdater;
            std::shared_ptr<diagnostic_updater::FrequencyStatus> createFrequencyStatus();

            std::vector<ros::Publisher> wrappedPublishers;

            std::shared_ptr<ros::Publisher> flagsPublisher;

            double loopFrequency;

            /**
             * @brief Publish the flags as ROS message.
             */
            void publishFlags() const;

        protected:

            AbstractNodeSm sm;

            /**
             * @brief Called when trying to set the node into state NODE_CONFIGURED.
             *        If this method returns false, the transition is not executed and the node
             *        stays in the state it has been in before.
             * 
             * @return True if transition is possible, else false.
             */
            virtual bool doInit() = 0;

            /**
             * @brief Called when trying to set the node into state STOPPED.
             *        If this method returns false, the transition is not executed and the node
             *        stays in the state it has been in before.
             * 
             * @return True if transition is possible, else false.
             */
            virtual bool doStop() = 0;

            /**
             * @brief Called when trying to set the node into state COMPONENT_CONNECTED.
             *        If this method returns false, the transition is not executed and the node
             *        stays in the state it has been in before.
             * 
             * @return True if transition is possible, else false.
             */
            virtual bool doConnect() = 0;

            /**
             * @brief Called when trying to set the node into state COMPONENT_DISCONNECTED.
             *        If this method returns false, the transition is not executed and the node
             *        stays in the state it has been in before.
             * 
             * @return True if transition is possible, else false.
             */
            virtual bool doDisconnect() = 0;

            /**
             * @brief Called when trying to set the node into state COMPONENT_PAUSED.
             *        If this method returns false, the transition is not executed and the node
             *        stays in the state it has been in before.
             * 
             * @return True if transition is possible, else false.
             */
            virtual bool doPause() = 0;

            /**
             * @brief Called when trying to set the node into state COMPONENT_RUNNING (from COMPONENT_PAUSED).
             *        If this method returns false, the transition is not executed and the node
             *        stays in the state it has been in before.
             * 
             * @return True if transition is possible, else false.
             */
            virtual bool doResume() = 0;

            /**
             * @brief Called when trying to set the node into state COMPONENT_RUNNING or COMPONENT_PAUSED (from COMPONENT_CONNECTED).
             *        If this method returns false, the transition is not executed and the node
             *        stays in the state it has been in before.
             * 
             * @param running If true, the component should be set to COMPONENT_RUNNING, else to COMPONENT_PAUSED.
             * 
             * @return True if transition is possible, else false.
             */
            virtual bool doStart(bool& running) = 0;

            /**
             * @brief Override this method if your code needs to be called periodically (e.g. to publish status messages).
             *        This method will be called in the node's main loop independent of the
             *        node's state. The default frequency is 10 Hz.
             */
            virtual void doStep() = 0;
            
            /**
             * @brief Executes the desired transition, if possible.
             *        If no transition is set, nothing happens.
             * 
             * @param currentStatePublisher If this is not a nullpointer, the new current state will be published by the currentStatePublisher
             */
            void doTransition(const std::unique_ptr<ros::Publisher>& currentStatePublisher);

            std::shared_ptr<ros::NodeHandle> nodeHandlePrivate;

            /**
             * @brief Get a reference to the diagnostic_updater.
             *        You should call setHardwareID[f] on it in your constructor and update it
             *        when you connected to real hardware and got more information (e.g. serial number).
             *        The updater is called automatically in the node's loop.
             * 
             * @return Reference to a diagnostic_updater::Updater object
             */
            std::shared_ptr<diagnostic_updater::Updater> getDiagnosticUpdater();


            /**
             * @brief Gets the frequency the node is configured to loop with.
             *        It defines with which frequency the node performs transitions and calls doStep().
             *        Is configurable by the ROS parameter ~/loop_frequency.
             * 
             * @return The frequency the node is configured to loop with.
             */
            double getLoopFrequency();

            /**
             * @brief Create a diagnosed publisher that is inspected by ros diagnositcs for the specified arguments.
             *        Only returns the diagnosed publisher. Stores the wrapped publisher in the internal container wrappedPublishers.
             * 
             * @param topic            The topic the diagnosed publisher should be created for.
             * @param queueSize        The queue size used for the publisher.
             * @param latch            Indicates whether the topic should be latched or not.
             * @param freqStatus       A reference to the expected frequency configuration of the publisher.
             * @param timestampStatus  A reference to the expected time stamp configuration of the publisher.
             * 
             * @return A diagnosed publisher configured with the specified arguments.
             */
            template<class T>
            std::shared_ptr<diagnostic_updater::DiagnosedPublisher<T>> createDiagnosedPublisher(
                const std::string& topic,
                uint32_t queueSize,
                bool latch,
                const diagnostic_updater::FrequencyStatusParam& freqStatus,
                const diagnostic_updater::TimeStampStatusParam& timestampStatus)
            {
                auto wrappedPublisher = nodeHandlePrivate->advertise<T>(topic, queueSize, latch);
                wrappedPublishers.push_back(wrappedPublisher);
                return std::make_shared<diagnostic_updater::DiagnosedPublisher<T>>(wrappedPublisher, *diagUpdater, freqStatus, timestampStatus);
            }

            /**
             * @brief Indicates whether the last transition has been executed successfuly.
             * 
             * @return True if transition was successful, else false.
             */
            bool getTransitionSuccessful() const;

    };
}

#endif
