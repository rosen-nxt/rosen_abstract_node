#ifndef ROSEN_ABSTRACT_NODE_TRANSITIONS_ROSTEST_H
#define ROSEN_ABSTRACT_NODE_TRANSITIONS_ROSTEST_H

#include <tuple>

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "rosen_abstract_node/NodeTransition.h"
#include "rosen_abstract_node/NodeState.h"
#include "rosen_abstract_node/StateTransitionAction.h"


namespace rosen_abstract_node::transitions_rostest
{
    int test_main(int argc, char** argv)
    {
        testing::InitGoogleTest(&argc, argv);
        ros::init(argc, argv, "transitions_rostest");
        ros::NodeHandle nh("~");
        auto test_filter = nh.param<std::string>("test_filter", "*");
        auto test_filter_expression = "transitions_rostest." + test_filter;
        testing::GTEST_FLAG(filter) = test_filter_expression;
        return RUN_ALL_TESTS();
    }

    std::tuple<actionlib::SimpleClientGoalState::StateEnum, uint8_t> do_transition(const uint8_t node_transition)
    {
        ros::NodeHandle nh("~");
        const auto action_namespace = nh.param<std::string>("action_namespace", "");
        const auto action_topic = action_namespace + "/state_transition_action";
        actionlib::SimpleActionClient<StateTransitionAction> action_client(action_topic, true);
        action_client.waitForServer();

        StateTransitionGoal goal;
        goal.transition = node_transition;
        const auto state = action_client.sendGoalAndWait(goal);
        const auto result = action_client.getResult();

        return std::make_tuple(state.state_, result->new_state);
    }

    void test_transition(const uint8_t transition, const actionlib::SimpleClientGoalState::StateEnum expected_goal_state, const uint8_t expected_state)
    {
        actionlib::SimpleClientGoalState::StateEnum goal_state;
        uint8_t new_state;
        std::tie(goal_state, new_state) = do_transition(transition);
        ASSERT_EQ(expected_goal_state, goal_state);
        ASSERT_EQ(expected_state, new_state);
    }

    void configure_node_behaviour(const std::string& method, const bool configuration)
    {
        ros::NodeHandle nh("~");
        const auto action_namespace = nh.param<std::string>("configuration_namespace", "");
        const auto parameter_name = action_namespace + "/dummy_node_" + method;
        nh.setParam(parameter_name, configuration);
    }

    void configure_do_init(const bool succeeds)
    {
        configure_node_behaviour("do_init_succeeds", succeeds);
    }

    void configure_do_stop(const bool succeeds)
    {
        configure_node_behaviour("do_stop_succeeds", succeeds);
    }

    void configure_do_connect(const bool succeeds)
    {
        configure_node_behaviour("do_connect_succeeds", succeeds);
    }

    void configure_do_disconnect(const bool succeeds)
    {
        configure_node_behaviour("do_disconnect_succeeds", succeeds);
    }

    void configure_do_pause(const bool succeeds)
    {
        configure_node_behaviour("do_pause_succeeds", succeeds);
    }

    void configure_do_start(const bool succeeds, const bool running)
    {
        configure_node_behaviour("do_start_succeeds", succeeds);
        configure_node_behaviour("running", running);
    }

    void configure_do_resume(const bool succeeds)
    {
        configure_node_behaviour("do_resume_succeeds", succeeds);
    }

    void set_node_to_state(const uint8_t desired_state)
    {
        actionlib::SimpleClientGoalState::StateEnum action_state;
        uint8_t new_state = rosen_abstract_node::NodeState::STOPPED;

        switch(desired_state)
        {
            case rosen_abstract_node::NodeState::STOPPED:
                break;
            case rosen_abstract_node::NodeState::NODE_CONFIGURED:
                std::tie(action_state, new_state) = do_transition(rosen_abstract_node::NodeTransition::INIT);
                break;
            case rosen_abstract_node::NodeState::COMPONENT_CONNECTED:
                std::tie(action_state, new_state) = do_transition(rosen_abstract_node::NodeTransition::INIT);
                std::tie(action_state, new_state) = do_transition(rosen_abstract_node::NodeTransition::CONNECT);
                break;
            case rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED:
                std::tie(action_state, new_state) = do_transition(rosen_abstract_node::NodeTransition::INIT);
                std::tie(action_state, new_state) = do_transition(rosen_abstract_node::NodeTransition::CONNECT);
                std::tie(action_state, new_state) = do_transition(rosen_abstract_node::NodeTransition::DISCONNECT);
                break;
            case rosen_abstract_node::NodeState::COMPONENT_RUNNING:
                std::tie(action_state, new_state) = do_transition(rosen_abstract_node::NodeTransition::INIT);
                std::tie(action_state, new_state) = do_transition(rosen_abstract_node::NodeTransition::CONNECT);
                std::tie(action_state, new_state) = do_transition(rosen_abstract_node::NodeTransition::START);
                break;
            case rosen_abstract_node::NodeState::COMPONENT_PAUSED:
                std::tie(action_state, new_state) = do_transition(rosen_abstract_node::NodeTransition::INIT);
                std::tie(action_state, new_state) = do_transition(rosen_abstract_node::NodeTransition::CONNECT);
                std::tie(action_state, new_state) = do_transition(rosen_abstract_node::NodeTransition::START);
                std::tie(action_state, new_state) = do_transition(rosen_abstract_node::NodeTransition::PAUSE);
                break;
            default:
                FAIL();
        }

        ASSERT_EQ(desired_state, new_state);
    }
}


/*
 * Invalid or unknown transitions
 */ 

TEST(transitions_rostest, goal_state_aborted_after_invalid_transition)
{ 
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::STOPPED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::CONNECT, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::STOPPED);
}

TEST(transitions_rostest, goal_state_aborted_after_unknown_transition)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::STOPPED);

    rosen_abstract_node::transitions_rostest::test_transition(UINT8_MAX, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::STOPPED);
}

/*
 * rosen_abstract_node::NodeState::STOPPED
 */ 

TEST(transitions_rostest, node_stopped_init_valid_success)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::STOPPED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::INIT, actionlib::SimpleClientGoalState::SUCCEEDED, rosen_abstract_node::NodeState::NODE_CONFIGURED);
}

TEST(transitions_rostest, node_stopped_init_valid_fail)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::STOPPED);
    rosen_abstract_node::transitions_rostest::configure_do_init(false);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::INIT, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::STOPPED);
}

TEST(transitions_rostest, node_stopped_connect_invalid)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::STOPPED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::CONNECT, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::STOPPED);
}

TEST(transitions_rostest, node_stopped_disconnect_invalid)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::STOPPED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::DISCONNECT, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::STOPPED);
}

TEST(transitions_rostest, node_stopped_start_invalid)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::STOPPED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::START, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::STOPPED);
}

TEST(transitions_rostest, node_stopped_pause_invalid)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::STOPPED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::PAUSE, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::STOPPED);
}

TEST(transitions_rostest, node_stopped_resume_invalid)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::STOPPED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::RESUME, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::STOPPED);
}

TEST(transitions_rostest, node_stopped_stop_invalid)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::STOPPED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::STOP, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::STOPPED);
}

/*
 * rosen_abstract_node::NodeState::NODE_CONFIGURED
 */ 

TEST(transitions_rostest, node_configured_init_invalid)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::NODE_CONFIGURED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::INIT, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::NODE_CONFIGURED);
}

TEST(transitions_rostest, node_configured_connect_valid_success)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::NODE_CONFIGURED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::CONNECT, actionlib::SimpleClientGoalState::SUCCEEDED, rosen_abstract_node::NodeState::COMPONENT_CONNECTED);
}

TEST(transitions_rostest, node_configured_connect_valid_fail)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::NODE_CONFIGURED);

    rosen_abstract_node::transitions_rostest::configure_do_connect(false);
    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::CONNECT, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::NODE_CONFIGURED);
}

TEST(transitions_rostest, node_configured_disconnect_invalid)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::NODE_CONFIGURED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::DISCONNECT, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::NODE_CONFIGURED);
}

TEST(transitions_rostest, node_configured_start_invalid)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::NODE_CONFIGURED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::START, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::NODE_CONFIGURED);
}

TEST(transitions_rostest, node_configured_pause_invalid)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::NODE_CONFIGURED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::START, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::NODE_CONFIGURED);
}

TEST(transitions_rostest, node_configured_resume_invalid)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::NODE_CONFIGURED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::RESUME, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::NODE_CONFIGURED);
}

TEST(transitions_rostest, node_configured_stop_valid_success)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::NODE_CONFIGURED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::STOP, actionlib::SimpleClientGoalState::SUCCEEDED, rosen_abstract_node::NodeState::STOPPED);
}

TEST(transitions_rostest, node_configured_stop_valid_fail)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::NODE_CONFIGURED);

    rosen_abstract_node::transitions_rostest::configure_do_stop(false);
    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::STOP, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::NODE_CONFIGURED);
}

/*
 * rosen_abstract_node::NodeState::COMPONENT_CONNECTED
 */ 

TEST(transitions_rostest, node_connected_init_invalid)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_CONNECTED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::INIT, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_CONNECTED);
}

TEST(transitions_rostest, node_connected_connect_invalid)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_CONNECTED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::CONNECT, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_CONNECTED);
}

TEST(transitions_rostest, node_connected_disconnect_valid_success)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_CONNECTED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::DISCONNECT, actionlib::SimpleClientGoalState::SUCCEEDED, rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED);
}

TEST(transitions_rostest, node_connected_disconnect_valid_fail)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_CONNECTED);

    rosen_abstract_node::transitions_rostest::configure_do_disconnect(false);
    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::DISCONNECT, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_CONNECTED);
}

TEST(transitions_rostest, node_connected_start_valid_running_success)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_CONNECTED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::START, actionlib::SimpleClientGoalState::SUCCEEDED, rosen_abstract_node::NodeState::COMPONENT_RUNNING);
}

TEST(transitions_rostest, node_connected_start_valid_running_fail)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_CONNECTED);

    rosen_abstract_node::transitions_rostest::configure_do_start(false, true);
    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::START, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_CONNECTED);
}

TEST(transitions_rostest, node_connected_start_valid_paused_success)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_CONNECTED);

    rosen_abstract_node::transitions_rostest::configure_do_start(true, false);
    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::START, actionlib::SimpleClientGoalState::SUCCEEDED, rosen_abstract_node::NodeState::COMPONENT_PAUSED);
}

TEST(transitions_rostest, node_connected_start_valid_paused_fail)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_CONNECTED);

    rosen_abstract_node::transitions_rostest::configure_do_start(false, false);
    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::START, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_CONNECTED);
}

TEST(transitions_rostest, node_connected_pause_invalid)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_CONNECTED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::PAUSE, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_CONNECTED);
}

TEST(transitions_rostest, node_connected_resume_invalid)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_CONNECTED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::RESUME, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_CONNECTED);
}

TEST(transitions_rostest, node_connected_stop_invalid)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_CONNECTED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::STOP, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_CONNECTED);
}

/*
 * rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED
 */ 

TEST(transitions_rostest, node_disconnected_init_invalid)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::INIT, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED);
}

TEST(transitions_rostest, node_disconnected_connect_valid_success)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::CONNECT, actionlib::SimpleClientGoalState::SUCCEEDED, rosen_abstract_node::NodeState::COMPONENT_CONNECTED);
}

TEST(transitions_rostest, node_disconnected_connect_valid_fail)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED);

    rosen_abstract_node::transitions_rostest::configure_do_connect(false);
    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::CONNECT, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED);
}

TEST(transitions_rostest, node_disconnected_disconnect_invalid)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::DISCONNECT, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED);
}

TEST(transitions_rostest, node_disconnected_start_invalid)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::START, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED);
}

TEST(transitions_rostest, node_disconnected_pause_invalid)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::PAUSE, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED);
}

TEST(transitions_rostest, node_disconnected_resume_invalid)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::RESUME, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED);
}

TEST(transitions_rostest, node_disconnected_stop_valid_success)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::STOP, actionlib::SimpleClientGoalState::SUCCEEDED, rosen_abstract_node::NodeState::STOPPED);
}

TEST(transitions_rostest, node_disconnected_stop_valid_fail)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED);

    rosen_abstract_node::transitions_rostest::configure_do_stop(false);
    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::STOP, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED);
}

/*
 * rosen_abstract_node::NodeState::COMPONENT_RUNNING
 */ 

TEST(transitions_rostest, node_running_init_invalid)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_RUNNING);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::INIT, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_RUNNING);
}

TEST(transitions_rostest, node_running_connect_invalid)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_RUNNING);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::CONNECT, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_RUNNING);
}

TEST(transitions_rostest, node_running_disconnect_valid_success)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_RUNNING);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::DISCONNECT, actionlib::SimpleClientGoalState::SUCCEEDED, rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED);
}

TEST(transitions_rostest, node_running_disconnect_valid_fail)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_RUNNING);

    rosen_abstract_node::transitions_rostest::configure_do_disconnect(false);
    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::DISCONNECT, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_RUNNING);
}

TEST(transitions_rostest, node_running_start_invalid)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_RUNNING);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::START, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_RUNNING);
}

TEST(transitions_rostest, node_running_pause_valid_success)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_RUNNING);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::PAUSE, actionlib::SimpleClientGoalState::SUCCEEDED, rosen_abstract_node::NodeState::COMPONENT_PAUSED);
}

TEST(transitions_rostest, node_running_pause_valid_fail)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_RUNNING);

    rosen_abstract_node::transitions_rostest::configure_do_pause(false);
    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::PAUSE, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_RUNNING);
}

TEST(transitions_rostest, node_running_resume_invalid)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_RUNNING);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::RESUME, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_RUNNING);
}

TEST(transitions_rostest, node_running_stop_invalid)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_RUNNING);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::STOP, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_RUNNING);
}

TEST(transitions_rostest, node_running_stop_and_start_valid_success)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_RUNNING);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::DISCONNECT, actionlib::SimpleClientGoalState::SUCCEEDED, rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED);
    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::STOP, actionlib::SimpleClientGoalState::SUCCEEDED, rosen_abstract_node::NodeState::STOPPED);
    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::INIT, actionlib::SimpleClientGoalState::SUCCEEDED, rosen_abstract_node::NodeState::NODE_CONFIGURED);
    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::CONNECT, actionlib::SimpleClientGoalState::SUCCEEDED, rosen_abstract_node::NodeState::COMPONENT_CONNECTED);
    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::START, actionlib::SimpleClientGoalState::SUCCEEDED, rosen_abstract_node::NodeState::COMPONENT_RUNNING);
}

/*
 * rosen_abstract_node::NodeState::COMPONENT_PAUSED
 */ 

TEST(transitions_rostest, node_paused_init_invalid)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_PAUSED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::INIT, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_PAUSED);
}

TEST(transitions_rostest, node_paused_connect_invalid)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_PAUSED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::CONNECT, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_PAUSED);
}

TEST(transitions_rostest, node_paused_disconnect_valid_success)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_PAUSED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::DISCONNECT, actionlib::SimpleClientGoalState::SUCCEEDED, rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED);
}

TEST(transitions_rostest, node_paused_disconnect_valid_fail)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_PAUSED);

    rosen_abstract_node::transitions_rostest::configure_do_disconnect(false);
    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::DISCONNECT, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_PAUSED);
}

TEST(transitions_rostest, node_paused_start_invalid)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_PAUSED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::START, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_PAUSED);
}

TEST(transitions_rostest, node_paused_pause_invalid)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_PAUSED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::PAUSE, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_PAUSED);
}

TEST(transitions_rostest, node_paused_resume_valid_success)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_PAUSED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::RESUME, actionlib::SimpleClientGoalState::SUCCEEDED, rosen_abstract_node::NodeState::COMPONENT_RUNNING);
}

TEST(transitions_rostest, node_paused_resume_valid_fail)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_PAUSED);

    rosen_abstract_node::transitions_rostest::configure_do_resume(false);
    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::RESUME, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_PAUSED);
}

TEST(transitions_rostest, node_paused_stop_invalid)
{
    rosen_abstract_node::transitions_rostest::set_node_to_state(rosen_abstract_node::NodeState::COMPONENT_PAUSED);

    rosen_abstract_node::transitions_rostest::test_transition(rosen_abstract_node::NodeTransition::STOP, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_PAUSED);
}

#endif