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
    int testMain(int argc, char** argv)
    {
        testing::InitGoogleTest(&argc, argv);
        ros::init(argc, argv, "transitions_rostest");
        ros::NodeHandle nh("~");
        auto testFilter = nh.param<std::string>("test_filter", "*");
        auto testFilterExpression = "TransitionRostest." + testFilter;
        testing::GTEST_FLAG(filter) = testFilterExpression;
        return RUN_ALL_TESTS();
    }

    std::tuple<actionlib::SimpleClientGoalState::StateEnum, uint8_t> doTransition(const uint8_t nodeTransition)
    {
        ros::NodeHandle nh("~");
        const auto actionNamespace = nh.param<std::string>("action_namespace", "");
        const auto actionTopic = actionNamespace + "/state_transition_action";
        actionlib::SimpleActionClient<StateTransitionAction> actionClient(actionTopic, true);
        actionClient.waitForServer();

        StateTransitionGoal goal;
        goal.transition = nodeTransition;
        const auto state = actionClient.sendGoalAndWait(goal);
        const auto result = actionClient.getResult();

        return std::make_tuple(state.state_, result->new_state);
    }

    void testTransition(const uint8_t transition, const actionlib::SimpleClientGoalState::StateEnum expectedGoalState, const uint8_t expectedState)
    {
        actionlib::SimpleClientGoalState::StateEnum goalState;
        uint8_t newState;
        std::tie(goalState, newState) = doTransition(transition);
        ASSERT_EQ(expectedGoalState, goalState);
        ASSERT_EQ(expectedState, newState);
    }

    void configureNodeBehaviour(const std::string& method, const bool configuration)
    {
        ros::NodeHandle nh("~");
        const auto actionNamespace = nh.param<std::string>("configuration_namespace", "");
        const auto parameterName = actionNamespace + "/dummy_node_" + method;
        nh.setParam(parameterName, configuration);
    }

    void configureDoInit(const bool succeeds)
    {
        configureNodeBehaviour("do_init_succeeds", succeeds);
    }

    void configureDoStop(const bool succeeds)
    {
        configureNodeBehaviour("do_stop_succeeds", succeeds);
    }

    void configureDoConnect(const bool succeeds)
    {
        configureNodeBehaviour("do_connect_succeeds", succeeds);
    }

    void configureDoDisconnect(const bool succeeds)
    {
        configureNodeBehaviour("do_disconnect_succeeds", succeeds);
    }

    void configure_do_pause(const bool succeeds)
    {
        configureNodeBehaviour("do_pause_succeeds", succeeds);
    }

    void configureDoStart(const bool succeeds, const bool running)
    {
        configureNodeBehaviour("do_start_succeeds", succeeds);
        configureNodeBehaviour("running", running);
    }

    void configureDoResume(const bool succeeds)
    {
        configureNodeBehaviour("do_resume_succeeds", succeeds);
    }

    void setNodeToState(const uint8_t desiredState)
    {
        actionlib::SimpleClientGoalState::StateEnum actionState;
        uint8_t newState = rosen_abstract_node::NodeState::STOPPED;

        switch(desiredState)
        {
            case rosen_abstract_node::NodeState::STOPPED:
                break;
            case rosen_abstract_node::NodeState::NODE_CONFIGURED:
                std::tie(actionState, newState) = doTransition(rosen_abstract_node::NodeTransition::INIT);
                break;
            case rosen_abstract_node::NodeState::COMPONENT_CONNECTED:
                std::tie(actionState, newState) = doTransition(rosen_abstract_node::NodeTransition::INIT);
                std::tie(actionState, newState) = doTransition(rosen_abstract_node::NodeTransition::CONNECT);
                break;
            case rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED:
                std::tie(actionState, newState) = doTransition(rosen_abstract_node::NodeTransition::INIT);
                std::tie(actionState, newState) = doTransition(rosen_abstract_node::NodeTransition::CONNECT);
                std::tie(actionState, newState) = doTransition(rosen_abstract_node::NodeTransition::DISCONNECT);
                break;
            case rosen_abstract_node::NodeState::COMPONENT_RUNNING:
                std::tie(actionState, newState) = doTransition(rosen_abstract_node::NodeTransition::INIT);
                std::tie(actionState, newState) = doTransition(rosen_abstract_node::NodeTransition::CONNECT);
                std::tie(actionState, newState) = doTransition(rosen_abstract_node::NodeTransition::START);
                break;
            case rosen_abstract_node::NodeState::COMPONENT_PAUSED:
                std::tie(actionState, newState) = doTransition(rosen_abstract_node::NodeTransition::INIT);
                std::tie(actionState, newState) = doTransition(rosen_abstract_node::NodeTransition::CONNECT);
                std::tie(actionState, newState) = doTransition(rosen_abstract_node::NodeTransition::START);
                std::tie(actionState, newState) = doTransition(rosen_abstract_node::NodeTransition::PAUSE);
                break;
            default:
                FAIL();
        }

        ASSERT_EQ(desiredState, newState);
    }
}


/*
 * Invalid or unknown transitions
 */ 

TEST(TransitionRostest, goalStateAbortedAfterUnknownTransition)
{
    rosen_abstract_node::transitions_rostest::setNodeToState(rosen_abstract_node::NodeState::STOPPED);

    rosen_abstract_node::transitions_rostest::testTransition(UINT8_MAX, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::STOPPED);
}

/*
 * rosen_abstract_node::NodeState::STOPPED
 */ 

TEST(TransitionRostest, nodeStoppedInitValidSuccess)
{
    rosen_abstract_node::transitions_rostest::setNodeToState(rosen_abstract_node::NodeState::STOPPED);

    rosen_abstract_node::transitions_rostest::testTransition(rosen_abstract_node::NodeTransition::INIT, actionlib::SimpleClientGoalState::SUCCEEDED, rosen_abstract_node::NodeState::NODE_CONFIGURED);
}

TEST(TransitionRostest, nodeStoppedInitValidFail)
{
    rosen_abstract_node::transitions_rostest::setNodeToState(rosen_abstract_node::NodeState::STOPPED);
    rosen_abstract_node::transitions_rostest::configureDoInit(false);

    rosen_abstract_node::transitions_rostest::testTransition(rosen_abstract_node::NodeTransition::INIT, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::STOPPED);
}

/*
 * rosen_abstract_node::NodeState::NODE_CONFIGURED
 */ 

TEST(TransitionRostest, nodeConfiguredConnectValidSuccess)
{
    rosen_abstract_node::transitions_rostest::setNodeToState(rosen_abstract_node::NodeState::NODE_CONFIGURED);

    rosen_abstract_node::transitions_rostest::testTransition(rosen_abstract_node::NodeTransition::CONNECT, actionlib::SimpleClientGoalState::SUCCEEDED, rosen_abstract_node::NodeState::COMPONENT_CONNECTED);
}

TEST(TransitionRostest, nodeConfiguredConnectValidFail)
{
    rosen_abstract_node::transitions_rostest::setNodeToState(rosen_abstract_node::NodeState::NODE_CONFIGURED);

    rosen_abstract_node::transitions_rostest::configureDoConnect(false);
    rosen_abstract_node::transitions_rostest::testTransition(rosen_abstract_node::NodeTransition::CONNECT, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::NODE_CONFIGURED);
}

TEST(TransitionRostest, nodeConfiguredStopValidSuccess)
{
    rosen_abstract_node::transitions_rostest::setNodeToState(rosen_abstract_node::NodeState::NODE_CONFIGURED);

    rosen_abstract_node::transitions_rostest::testTransition(rosen_abstract_node::NodeTransition::STOP, actionlib::SimpleClientGoalState::SUCCEEDED, rosen_abstract_node::NodeState::STOPPED);
}

TEST(TransitionRostest, nodeConfiguredStopValidFail)
{
    rosen_abstract_node::transitions_rostest::setNodeToState(rosen_abstract_node::NodeState::NODE_CONFIGURED);

    rosen_abstract_node::transitions_rostest::configureDoStop(false);
    rosen_abstract_node::transitions_rostest::testTransition(rosen_abstract_node::NodeTransition::STOP, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::NODE_CONFIGURED);
}

/*
 * rosen_abstract_node::NodeState::COMPONENT_CONNECTED
 */ 

TEST(TransitionRostest, nodeConnectedDisconnectValidSuccess)
{
    rosen_abstract_node::transitions_rostest::setNodeToState(rosen_abstract_node::NodeState::COMPONENT_CONNECTED);

    rosen_abstract_node::transitions_rostest::testTransition(rosen_abstract_node::NodeTransition::DISCONNECT, actionlib::SimpleClientGoalState::SUCCEEDED, rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED);
}

TEST(TransitionRostest, nodeConnectedDisconnectValidFail)
{
    rosen_abstract_node::transitions_rostest::setNodeToState(rosen_abstract_node::NodeState::COMPONENT_CONNECTED);

    rosen_abstract_node::transitions_rostest::configureDoDisconnect(false);
    rosen_abstract_node::transitions_rostest::testTransition(rosen_abstract_node::NodeTransition::DISCONNECT, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_CONNECTED);
}

TEST(TransitionRostest, nodeConnectedStartValidRunningSuccess)
{
    rosen_abstract_node::transitions_rostest::setNodeToState(rosen_abstract_node::NodeState::COMPONENT_CONNECTED);

    rosen_abstract_node::transitions_rostest::testTransition(rosen_abstract_node::NodeTransition::START, actionlib::SimpleClientGoalState::SUCCEEDED, rosen_abstract_node::NodeState::COMPONENT_RUNNING);
}

TEST(TransitionRostest, nodeConnectedStartValidRunningFail)
{
    rosen_abstract_node::transitions_rostest::setNodeToState(rosen_abstract_node::NodeState::COMPONENT_CONNECTED);

    rosen_abstract_node::transitions_rostest::configureDoStart(false, true);
    rosen_abstract_node::transitions_rostest::testTransition(rosen_abstract_node::NodeTransition::START, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_CONNECTED);
}

TEST(TransitionRostest, nodeConnectedStartValidPausedSuccess)
{
    rosen_abstract_node::transitions_rostest::setNodeToState(rosen_abstract_node::NodeState::COMPONENT_CONNECTED);

    rosen_abstract_node::transitions_rostest::configureDoStart(true, false);
    rosen_abstract_node::transitions_rostest::testTransition(rosen_abstract_node::NodeTransition::START, actionlib::SimpleClientGoalState::SUCCEEDED, rosen_abstract_node::NodeState::COMPONENT_PAUSED);
}

TEST(TransitionRostest, nodeConnectedStartValidPausedFail)
{
    rosen_abstract_node::transitions_rostest::setNodeToState(rosen_abstract_node::NodeState::COMPONENT_CONNECTED);

    rosen_abstract_node::transitions_rostest::configureDoStart(false, false);
    rosen_abstract_node::transitions_rostest::testTransition(rosen_abstract_node::NodeTransition::START, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_CONNECTED);
}

/*
 * rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED
 */ 

TEST(TransitionRostest, nodeDisconnectedConnectValidSuccess)
{
    rosen_abstract_node::transitions_rostest::setNodeToState(rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED);

    rosen_abstract_node::transitions_rostest::testTransition(rosen_abstract_node::NodeTransition::CONNECT, actionlib::SimpleClientGoalState::SUCCEEDED, rosen_abstract_node::NodeState::COMPONENT_CONNECTED);
}

TEST(TransitionRostest, nodeDisconnectedConnectValidFail)
{
    rosen_abstract_node::transitions_rostest::setNodeToState(rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED);

    rosen_abstract_node::transitions_rostest::configureDoConnect(false);
    rosen_abstract_node::transitions_rostest::testTransition(rosen_abstract_node::NodeTransition::CONNECT, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED);
}

TEST(TransitionRostest, nodeDisconnectedStopValidSuccess)
{
    rosen_abstract_node::transitions_rostest::setNodeToState(rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED);

    rosen_abstract_node::transitions_rostest::testTransition(rosen_abstract_node::NodeTransition::STOP, actionlib::SimpleClientGoalState::SUCCEEDED, rosen_abstract_node::NodeState::STOPPED);
}

TEST(TransitionRostest, nodeDisconnectedStopValidFail)
{
    rosen_abstract_node::transitions_rostest::setNodeToState(rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED);

    rosen_abstract_node::transitions_rostest::configureDoStop(false);
    rosen_abstract_node::transitions_rostest::testTransition(rosen_abstract_node::NodeTransition::STOP, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED);
}

/*
 * rosen_abstract_node::NodeState::COMPONENT_RUNNING
 */ 

TEST(TransitionRostest, nodeRunningDisconnectValidSuccess)
{
    rosen_abstract_node::transitions_rostest::setNodeToState(rosen_abstract_node::NodeState::COMPONENT_RUNNING);

    rosen_abstract_node::transitions_rostest::testTransition(rosen_abstract_node::NodeTransition::DISCONNECT, actionlib::SimpleClientGoalState::SUCCEEDED, rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED);
}

TEST(TransitionRostest, nodeRunningDisconnectValidFail)
{
    rosen_abstract_node::transitions_rostest::setNodeToState(rosen_abstract_node::NodeState::COMPONENT_RUNNING);

    rosen_abstract_node::transitions_rostest::configureDoDisconnect(false);
    rosen_abstract_node::transitions_rostest::testTransition(rosen_abstract_node::NodeTransition::DISCONNECT, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_RUNNING);
}

TEST(TransitionRostest, nodeRunningPauseValidSuccess)
{
    rosen_abstract_node::transitions_rostest::setNodeToState(rosen_abstract_node::NodeState::COMPONENT_RUNNING);

    rosen_abstract_node::transitions_rostest::testTransition(rosen_abstract_node::NodeTransition::PAUSE, actionlib::SimpleClientGoalState::SUCCEEDED, rosen_abstract_node::NodeState::COMPONENT_PAUSED);
}

TEST(TransitionRostest, nodeRunningPauseValidFail)
{
    rosen_abstract_node::transitions_rostest::setNodeToState(rosen_abstract_node::NodeState::COMPONENT_RUNNING);

    rosen_abstract_node::transitions_rostest::configure_do_pause(false);
    rosen_abstract_node::transitions_rostest::testTransition(rosen_abstract_node::NodeTransition::PAUSE, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_RUNNING);
}

TEST(TransitionRostest, nodeRunningStopAndStartValidSuccess)
{
    rosen_abstract_node::transitions_rostest::setNodeToState(rosen_abstract_node::NodeState::COMPONENT_RUNNING);

    rosen_abstract_node::transitions_rostest::testTransition(rosen_abstract_node::NodeTransition::DISCONNECT, actionlib::SimpleClientGoalState::SUCCEEDED, rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED);
    rosen_abstract_node::transitions_rostest::testTransition(rosen_abstract_node::NodeTransition::STOP, actionlib::SimpleClientGoalState::SUCCEEDED, rosen_abstract_node::NodeState::STOPPED);
    rosen_abstract_node::transitions_rostest::testTransition(rosen_abstract_node::NodeTransition::INIT, actionlib::SimpleClientGoalState::SUCCEEDED, rosen_abstract_node::NodeState::NODE_CONFIGURED);
    rosen_abstract_node::transitions_rostest::testTransition(rosen_abstract_node::NodeTransition::CONNECT, actionlib::SimpleClientGoalState::SUCCEEDED, rosen_abstract_node::NodeState::COMPONENT_CONNECTED);
    rosen_abstract_node::transitions_rostest::testTransition(rosen_abstract_node::NodeTransition::START, actionlib::SimpleClientGoalState::SUCCEEDED, rosen_abstract_node::NodeState::COMPONENT_RUNNING);
}

/*
 * rosen_abstract_node::NodeState::COMPONENT_PAUSED
 */ 

TEST(TransitionRostest, nodePausedDisconnectValidSuccess)
{
    rosen_abstract_node::transitions_rostest::setNodeToState(rosen_abstract_node::NodeState::COMPONENT_PAUSED);

    rosen_abstract_node::transitions_rostest::testTransition(rosen_abstract_node::NodeTransition::DISCONNECT, actionlib::SimpleClientGoalState::SUCCEEDED, rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED);
}

TEST(TransitionRostest, nodePausedDisconnectValidFail)
{
    rosen_abstract_node::transitions_rostest::setNodeToState(rosen_abstract_node::NodeState::COMPONENT_PAUSED);

    rosen_abstract_node::transitions_rostest::configureDoDisconnect(false);
    rosen_abstract_node::transitions_rostest::testTransition(rosen_abstract_node::NodeTransition::DISCONNECT, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_PAUSED);
}

TEST(TransitionRostest, nodePausedResumeValidSuccess)
{
    rosen_abstract_node::transitions_rostest::setNodeToState(rosen_abstract_node::NodeState::COMPONENT_PAUSED);

    rosen_abstract_node::transitions_rostest::testTransition(rosen_abstract_node::NodeTransition::RESUME, actionlib::SimpleClientGoalState::SUCCEEDED, rosen_abstract_node::NodeState::COMPONENT_RUNNING);
}

TEST(TransitionRostest, nodePausedResumeValidFail)
{
    rosen_abstract_node::transitions_rostest::setNodeToState(rosen_abstract_node::NodeState::COMPONENT_PAUSED);

    rosen_abstract_node::transitions_rostest::configureDoResume(false);
    rosen_abstract_node::transitions_rostest::testTransition(rosen_abstract_node::NodeTransition::RESUME, actionlib::SimpleClientGoalState::ABORTED, rosen_abstract_node::NodeState::COMPONENT_PAUSED);
}

#endif