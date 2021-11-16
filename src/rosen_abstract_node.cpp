#include "rosen_abstract_node/rosen_abstract_node.h"

#include <boost/bind.hpp>
#include <stdexcept>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <diagnostic_updater/publisher.h>
#include <diagnostic_updater/update_functions.h>

#include "rosen_abstract_node/NodeTransition.h"
#include "rosen_abstract_node/NodeState.h"

namespace rosen_abstract_node
{
    namespace
    {
        constexpr double DEFAULT_LOOP_FREQUENCY = 10.0;
        const ros::Duration CURRENT_STATE_FREQUENCY = ros::Duration(1.0);
    }

    rosen_abstract_node::rosen_abstract_node(const unsigned char flags)
    : rosen_abstract_node(ros::this_node::getName(), "~", flags)
    {}

    rosen_abstract_node::rosen_abstract_node(const std::string& node_name, const std::string& node_namespace, const unsigned char flags)
    : rosen_abstract_node(node_name, std::make_shared<ros::NodeHandle>(node_namespace), std::make_shared<diagnostic_updater::Updater>(), flags)
    {}

    rosen_abstract_node::rosen_abstract_node(const std::string& node_name,
                                             const std::shared_ptr<ros::NodeHandle>& ros_node_handle_private,
                                             const std::shared_ptr<diagnostic_updater::Updater>& ros_diagnostic_updater,
                                             const unsigned char flags)
    : flags(flags),
      ros_node_name(node_name),
      next_trans(NodeTransition::NONE),
      state_transition_action_server(nullptr),
      diag_updater(ros_diagnostic_updater),
      wrapped_publishers(),
      flags_publisher(nullptr),
      loop_frequency(DEFAULT_LOOP_FREQUENCY),
      sm(std::bind(&rosen_abstract_node::do_init, this),
         std::bind(&rosen_abstract_node::do_connect, this),
         std::bind(&rosen_abstract_node::do_disconnect, this),
         std::bind(&rosen_abstract_node::do_start, this, std::placeholders::_1),
         std::bind(&rosen_abstract_node::do_pause, this),
         std::bind(&rosen_abstract_node::do_resume, this),
         std::bind(&rosen_abstract_node::do_stop, this)),
      node_handle_private(ros_node_handle_private)
    {
        if (ros_node_handle_private != nullptr)
        {
            state_transition_action_server = std::make_shared<actionlib::SimpleActionServer<StateTransitionAction>>(
              *ros_node_handle_private,
              "state_transition_action",
              [&](const StateTransitionGoalConstPtr& goal) { this->sm_action_cb(goal); },
              false);
            state_transition_action_server->start();

            flags_publisher = std::make_shared<ros::Publisher>(node_handle_private->advertise<std_msgs::UInt8>("flags", 1, true));
            publish_flags();
        }
        if (diag_updater != nullptr)
        {
            diag_updater->setHardwareID(node_name);
        }
    }
    
    void rosen_abstract_node::sm_action_cb(const StateTransitionGoalConstPtr& goal)
    {
        transition_processed = false;

        const node_transition_no trans = goal->transition;
        if (node_transition_helper::is_valid(trans) && trans != NodeTransition::NONE)
        {
            ROS_INFO_STREAM("abstract_node::sm_action_cb transition: " << node_transition_helper::to_string(trans));

            StateTransitionFeedback state_transition_feedback;
            state_transition_feedback.current_state = sm.get_current_state();
            state_transition_feedback.transition = trans;
            state_transition_action_server->publishFeedback(state_transition_feedback);

            initiate_transition(trans);

            std::unique_lock<std::mutex> lock(transition_mutex);
            transition_condition_variable.wait(lock, [&]{ return transition_processed; });
        }
        else
        {
            ROS_WARN("abstract_node::sm_action_cb received an invalid transition request: %i", trans);
            transition_successful = false;
        }

        StateTransitionResult state_transition_result;
        state_transition_result.new_state = sm.get_current_state();
        if (transition_successful)
        {
            state_transition_action_server->setSucceeded(state_transition_result);
        }
        else
        {
            state_transition_action_server->setAborted(state_transition_result);
        }
    }

    void rosen_abstract_node::initiate_transition(node_transition_no transition)
    {
        next_trans = transition;
    }

    void rosen_abstract_node::do_transition(const std::unique_ptr<ros::Publisher>& current_state_publisher)
    {
        if (next_trans == NodeTransition::NONE)
        {
            return;
        }

        {
            std::lock_guard<std::mutex> lock(transition_mutex);
            transition_successful = false;

            try
            {
                transition_successful = sm.do_transition(next_trans);
            }
            catch (const std::out_of_range&)
            {
                const auto message = "abstract_node::do_transition: Received invalid transition. Please check at caller!";
                ROS_FATAL(message);
                throw std::logic_error(message);
            }

            if (nullptr != current_state_publisher)
            {
                NodeStateInfo node_state_info;
                ros::Time now = ros::Time::now();
                rosen_abstract_node::update_and_publish_node_state_info(*current_state_publisher, now, node_state_info);
            }
            transition_processed = true;
            next_trans = NodeTransition::NONE;
        }

        transition_condition_variable.notify_one();
    }

    bool rosen_abstract_node::get_transition_successful() const
    {
        return transition_successful;
    }

    node_state_no rosen_abstract_node::get_current_state() const
    {
        return sm.get_current_state();
    }

    void rosen_abstract_node::update_and_publish_node_state_info(const ros::Publisher& current_state_publisher, const ros::Time now, NodeStateInfo& node_state_info)
    {
        node_state_info.current_state = sm.get_current_state();
        node_state_info.header.stamp = now;
        current_state_publisher.publish(node_state_info);
        ++node_state_info.header.seq;
    }

    std::shared_ptr<diagnostic_updater::FrequencyStatus> rosen_abstract_node::create_frequency_status()
    {
        auto frequency_param = diagnostic_updater::FrequencyStatusParam(&loop_frequency, &loop_frequency, 0.1, 10);
        auto frequency_status = std::make_shared<diagnostic_updater::FrequencyStatus>(frequency_param);
        diag_updater->add(*frequency_status);
        return frequency_status;
    }

    void rosen_abstract_node::loop()
    {
        loop_frequency = node_handle_private->param("loop_frequency", DEFAULT_LOOP_FREQUENCY);
        ROS_INFO_STREAM(ros_node_name << " is running with a frequency of " << loop_frequency << " Hz (ROS Time).");

        ros::Time now = ros::Time::now();
        ros::Rate loop_rate(loop_frequency);
        auto frequency_status = create_frequency_status();

        NodeStateInfo node_state_info;
        node_state_info.header.seq = 0;
        std::unique_ptr<ros::Publisher> current_state_publisher = std::make_unique<ros::Publisher>(node_handle_private->advertise<NodeStateInfo>("current_state", 10, false));

        ros::Time next_current_state_message = now;

        while (ros::ok())
        {
            now = ros::Time::now();
            ros::spinOnce();
            do_transition(current_state_publisher);

            if (now >= next_current_state_message)
            {
                update_and_publish_node_state_info(*current_state_publisher, now, node_state_info);
                next_current_state_message = now + CURRENT_STATE_FREQUENCY;
            }

            do_step();
            frequency_status->tick();
            diag_updater->update();
            loop_rate.sleep();
        }
    }

    std::shared_ptr<diagnostic_updater::Updater> rosen_abstract_node::get_diagnostic_updater()
    {
        return diag_updater;
    }

    double rosen_abstract_node::get_loop_frequency()
    {
        return loop_frequency;
    }

    unsigned char rosen_abstract_node::get_flags() const
    {
        return flags;
    }

    void rosen_abstract_node::publish_flags() const
    {
        std_msgs::UInt8 flags_msg;
        flags_msg.data = flags;
        flags_publisher->publish(flags_msg);
    }
}
