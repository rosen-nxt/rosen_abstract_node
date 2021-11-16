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
    class rosen_abstract_node
    {
        public:

            /**
             * @brief Initialises the abstract node with default node name and namespace. Only the flags are required.
             *
             * @param flags  Additional node control parameter. They can be used by other nodes to
             *               control this node in a certain way. The concrete definition of these parameters
             *               is not part of the abstract_node.
             */
            explicit rosen_abstract_node(const unsigned char flags);

            /**
             * @brief Initialises the abstract node with all required ROS components with the
             *        given node name in the given namespaces.
             *        Use this constructor if you want a default initialisation of all required ROS components.
             * 
             * @param node_name       The name of the ROS node.
             * @param node_namespace  The private namespace of the ROS node.
             * @param flags           Additional node control parameter. They can be used by other nodes to
             *                        control this node in a certain way. The concrete definition of these parameters
             *                        is not part of the abstract_node.
             */
            explicit rosen_abstract_node(const std::string& node_name = ros::this_node::getName(),
                                         const std::string& node_namespace = "~",
                                         const unsigned char flags = 0);

            /**
             * @brief Initialises the abstract node with the passed node_handle and diagnostics
             *        and names the ROS node by the given node name.
             *        Use this constructor if you created the ROS components externally or want to use it
             *        without ROS core running.
             * 
             * @param node_name               The name of the ROS node.
             * @param ros_node_handle_private The private node handle to be used.
             * @param ros_diagnostic_updater  The diagnostic updater to be used for diagnostic purposes.
             * @param flags                   Additional node control parameter. They can be used by other nodes to
             *                                control this node in a certain way. The concrete definition of these
             *                                parameters is not part of the abstract_node.
             */
            rosen_abstract_node(const std::string& node_name,
                                const std::shared_ptr<ros::NodeHandle>& ros_node_handle_private,
                                const std::shared_ptr<diagnostic_updater::Updater>& ros_diagnostic_updater,
                                const unsigned char flags = 0);

            virtual ~rosen_abstract_node() = default;

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
            void initiate_transition(node_transition_no transition);

            /**
             * @brief Initiate the given transition to be executed by the node in the next loop.
             *        If multiple transitions get passed before the loop executed one of them,
             *        only the last one gets executed.
             *
             * @param transition The transition to be executed.
             */
            void initiate_transition(unsigned char transition);

            /**
             * @brief Gets the current state the node is in.
             * 
             * @return The current state the node is in.
             */
            unsigned int get_current_state() const;

            /**
             * @brief Returns the configured flags.
             *
             * @return The flags.
             */
            unsigned char get_flags() const;

        private:

            unsigned char flags;

            /**
             * @brief Callback used when initiating a transition via the state_transition_action.
             * 
             * @param goal The goal of the transition.
             */
            void sm_action_cb(const StateTransitionGoalConstPtr& goal);

            const std::string ros_node_name;

            node_transition_no next_trans;

            bool transition_processed = false;
            bool transition_successful = false;
            std::mutex transition_mutex;
            std::condition_variable transition_condition_variable;

            std::shared_ptr<actionlib::SimpleActionServer<StateTransitionAction>> state_transition_action_server;

            /**
             * @brief Update the node_state_info and publish it via ROS.
             *
             * @param current_state_publisher The publisher of the current state.
             * @param now The current time.
             * @param node_state_info This is the node state, which will updated and published.
             */
            void update_and_publish_node_state_info(const ros::Publisher& current_state_publisher, const ros::Time now, NodeStateInfo& node_state_info);

            std::shared_ptr<diagnostic_updater::Updater> diag_updater;
            std::shared_ptr<diagnostic_updater::FrequencyStatus> create_frequency_status();

            std::vector<ros::Publisher> wrapped_publishers;

            std::shared_ptr<ros::Publisher> flags_publisher;

            double loop_frequency;

            /**
             * @brief Publish the flags as ROS message.
             */
            void publish_flags() const;

        protected:

            abstract_node_sm sm;

            /**
             * @brief Called when trying to set the node into state NODE_CONFIGURED.
             *        If this method returns false, the transition is not executed and the node
             *        stays in the state it has been in before.
             * 
             * @return True if transition is possible, else false.
             */
            virtual bool do_init() = 0;

            /**
             * @brief Called when trying to set the node into state STOPPED.
             *        If this method returns false, the transition is not executed and the node
             *        stays in the state it has been in before.
             * 
             * @return True if transition is possible, else false.
             */
            virtual bool do_stop() = 0;

            /**
             * @brief Called when trying to set the node into state COMPONENT_CONNECTED.
             *        If this method returns false, the transition is not executed and the node
             *        stays in the state it has been in before.
             * 
             * @return True if transition is possible, else false.
             */
            virtual bool do_connect() = 0;

            /**
             * @brief Called when trying to set the node into state COMPONENT_DISCONNECTED.
             *        If this method returns false, the transition is not executed and the node
             *        stays in the state it has been in before.
             * 
             * @return True if transition is possible, else false.
             */
            virtual bool do_disconnect() = 0;

            /**
             * @brief Called when trying to set the node into state COMPONENT_PAUSED.
             *        If this method returns false, the transition is not executed and the node
             *        stays in the state it has been in before.
             * 
             * @return True if transition is possible, else false.
             */
            virtual bool do_pause() = 0;

            /**
             * @brief Called when trying to set the node into state COMPONENT_RUNNING (from COMPONENT_PAUSED).
             *        If this method returns false, the transition is not executed and the node
             *        stays in the state it has been in before.
             * 
             * @return True if transition is possible, else false.
             */
            virtual bool do_resume() = 0;

            /**
             * @brief Called when trying to set the node into state COMPONENT_RUNNING or COMPONENT_PAUSED (from COMPONENT_CONNECTED).
             *        If this method returns false, the transition is not executed and the node
             *        stays in the state it has been in before.
             * 
             * @param running If true, the component should be set to COMPONENT_RUNNING, else to COMPONENT_PAUSED.
             * 
             * @return True if transition is possible, else false.
             */
            virtual bool do_start(bool& running) = 0;

            /**
             * @brief Override this method if your code needs to be called periodically (e.g. to publish status messages).
             *        This method will be called in the node's main loop independent of the
             *        node's state. The default frequency is 10 Hz.
             */
            virtual void do_step() = 0;
            
            /**
             * @brief Executes the desired transition, if possible.
             *        If no transition is set, nothing happens.
             * 
             * @param current_state_publisher If this is not a nullpointer, the new current state will be published by the current_state_publisher
             */
            void do_transition(const std::unique_ptr<ros::Publisher>& current_state_publisher);

            std::shared_ptr<ros::NodeHandle> node_handle_private;

            /**
             * @brief Get a reference to the diagnostic_updater.
             *        You should call setHardwareID[f] on it in your constructor and update it
             *        when you connected to real hardware and got more information (e.g. serial number).
             *        The updater is called automatically in the node's loop.
             * 
             * @return Reference to a diagnostic_updater::Updater object
             */
            std::shared_ptr<diagnostic_updater::Updater> get_diagnostic_updater();


            /**
             * @brief Gets the frequency the node is configured to loop with.
             *        It defines with which frequency the node performs transitions and calls do_step().
             *        Is configurable by the ROS parameter ~/loop_frequency.
             * 
             * @return The frequency the node is configured to loop with.
             */
            double get_loop_frequency();

            /**
             * @brief Create a diagnosed publisher that is inspected by ros diagnositcs for the specified arguments.
             *        Only returns the diagnosed publisher. Stores the wrapped publisher in the internal container wrapped_publishers.
             * 
             * @param topic            The topic the diagnosed publisher should be created for.
             * @param queue_size       The queue size used for the publisher.
             * @param latch            Indicates whether the topic should be latched or not.
             * @param freq_status      A reference to the expected frequency configuration of the publisher.
             * @param timestamp_status A reference to the expected time stamp configuration of the publisher.
             * 
             * @return A diagnosed publisher configured with the specified arguments.
             */
            template<class T>
            std::shared_ptr<diagnostic_updater::DiagnosedPublisher<T>> create_diagnosed_publisher(
                const std::string& topic,
                uint32_t queue_size,
                bool latch,
                const diagnostic_updater::FrequencyStatusParam& freq_status,
                const diagnostic_updater::TimeStampStatusParam& timestamp_status)
            {
                auto wrapped_publisher = node_handle_private->advertise<T>(topic, queue_size, latch);
                wrapped_publishers.push_back(wrapped_publisher);
                return std::make_shared<diagnostic_updater::DiagnosedPublisher<T>>(wrapped_publisher, *diag_updater, freq_status, timestamp_status);
            }

            /**
             * @brief Indicates whether the last transition has been executed successfuly.
             * 
             * @return True if transition was successful, else false.
             */
            bool get_transition_successful() const;

    };
}

#endif
