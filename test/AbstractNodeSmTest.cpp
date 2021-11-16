#include "rosen_abstract_node/AbstractNodeSm.h"
#include "rosen_abstract_node/NodeStateHelper.h"
#include "rosen_abstract_node/NodeTransitionHelper.h"

#include <gtest/gtest.h>

using namespace rosen_abstract_node;
using namespace testing;


std::function<bool()> cb(const bool returnValue)
{
    return [returnValue](){return returnValue;};
}

std::function<bool(bool&)> startCb(const bool runningArg, const bool returnValue)
{
    return [runningArg, returnValue](bool& running){running = runningArg; return returnValue;};
}

TEST(AbstractNodeSmTest, initialState)
{
    AbstractNodeSm sm;
    ASSERT_EQ(NodeStateNo::STOPPED, sm.getCurrentState());
}

TEST(AbstractNodeSmTest, nodeConfiguredAfterInitIfSuccessful)
{
    AbstractNodeSm sm;

    sm.doTransition(NodeTransitionNo::INIT);
    ASSERT_EQ(NodeStateNo::NODE_CONFIGURED, sm.getCurrentState());
}

TEST(AbstractNodeSmTest, nodeStillStoppedAfterInitIfFailed)
{
    AbstractNodeSm sm(cb(false));

    sm.doTransition(NodeTransitionNo::INIT);
    ASSERT_EQ(NodeStateNo::STOPPED, sm.getCurrentState());
}

TEST(AbstractNodeSmTest, nodeConnectedAfterConnectIfSuccessful)
{
    AbstractNodeSm sm;

    sm.doTransition(NodeTransitionNo::INIT);
    ASSERT_EQ(NodeStateNo::NODE_CONFIGURED, sm.getCurrentState());

    sm.doTransition(NodeTransitionNo::CONNECT);
    ASSERT_EQ(NodeStateNo::COMPONENT_CONNECTED, sm.getCurrentState());
}

TEST(AbstractNodeSmTest, stillConfiguredAfterConnectIfFailed)
{
    AbstractNodeSm sm(cb(true), cb(false));

    sm.doTransition(NodeTransitionNo::INIT);
    ASSERT_EQ(NodeStateNo::NODE_CONFIGURED, sm.getCurrentState());

    sm.doTransition(NodeTransitionNo::CONNECT);
    ASSERT_EQ(NodeStateNo::NODE_CONFIGURED, sm.getCurrentState());
}

TEST(AbstractNodeSmTest, nodeRunningAfterStartIfSuccessfulAndNotPaused)
{
    AbstractNodeSm sm;

    sm.doTransition(NodeTransitionNo::INIT);
    sm.doTransition(NodeTransitionNo::CONNECT);
    ASSERT_EQ(NodeStateNo::COMPONENT_CONNECTED, sm.getCurrentState());

    sm.doTransition(NodeTransitionNo::START);
    ASSERT_EQ(NodeStateNo::COMPONENT_RUNNING, sm.getCurrentState());
}

TEST(AbstractNodeSmTest, nodePausedAfterStartIfSuccessfulAndPaused)
{
    AbstractNodeSm sm(cb(true), cb(true), cb(true), startCb(false, true));

    sm.doTransition(NodeTransitionNo::INIT);
    sm.doTransition(NodeTransitionNo::CONNECT);
    ASSERT_EQ(NodeStateNo::COMPONENT_CONNECTED, sm.getCurrentState());

    sm.doTransition(NodeTransitionNo::START);
    ASSERT_EQ(NodeStateNo::COMPONENT_PAUSED, sm.getCurrentState());
}

TEST(AbstractNodeSmTest, nodePausedWhenRunningAndPauseTriggered)
{
    AbstractNodeSm sm;

    sm.doTransition(NodeTransitionNo::INIT);
    sm.doTransition(NodeTransitionNo::CONNECT);
    sm.doTransition(NodeTransitionNo::START);
    ASSERT_EQ(NodeStateNo::COMPONENT_RUNNING, sm.getCurrentState());

    sm.doTransition(NodeTransitionNo::PAUSE);
    ASSERT_EQ(NodeStateNo::COMPONENT_PAUSED, sm.getCurrentState());
}

TEST(AbstractNodeSmTest, nodeStillRunningWhenRunningAndPauseFailed)
{
    AbstractNodeSm sm(cb(true), cb(true), cb(true), startCb(true, true), cb(false));

    sm.doTransition(NodeTransitionNo::INIT);
    sm.doTransition(NodeTransitionNo::CONNECT);
    sm.doTransition(NodeTransitionNo::START);
    ASSERT_EQ(NodeStateNo::COMPONENT_RUNNING, sm.getCurrentState());

    sm.doTransition(NodeTransitionNo::PAUSE);
    ASSERT_EQ(NodeStateNo::COMPONENT_RUNNING, sm.getCurrentState());
}

TEST(AbstractNodeSmTest, nodeRunningWhenPausedAndResumeTriggered)
{
    AbstractNodeSm sm(cb(true), cb(true), cb(true), startCb(false, true));

    sm.doTransition(NodeTransitionNo::INIT);
    sm.doTransition(NodeTransitionNo::CONNECT);
    sm.doTransition(NodeTransitionNo::START);
    ASSERT_EQ(NodeStateNo::COMPONENT_PAUSED, sm.getCurrentState());

    sm.doTransition(NodeTransitionNo::RESUME);
    ASSERT_EQ(NodeStateNo::COMPONENT_RUNNING, sm.getCurrentState());
}

TEST(AbstractNodeSmTest, nodeStillPausedWhenPausedAndResumeFailed)
{
    AbstractNodeSm sm(cb(true), cb(true), cb(true), startCb(false, true), cb(true), cb(false));

    sm.doTransition(NodeTransitionNo::INIT);
    sm.doTransition(NodeTransitionNo::CONNECT);
    sm.doTransition(NodeTransitionNo::START);
    ASSERT_EQ(NodeStateNo::COMPONENT_PAUSED, sm.getCurrentState());

    sm.doTransition(NodeTransitionNo::RESUME);
    ASSERT_EQ(NodeStateNo::COMPONENT_PAUSED, sm.getCurrentState());
}

TEST(AbstractNodeSmTest, nodeDisconnectedWhenRunningAndDisconnectTriggered)
{
    AbstractNodeSm sm;

    sm.doTransition(NodeTransitionNo::INIT);
    sm.doTransition(NodeTransitionNo::CONNECT);
    sm.doTransition(NodeTransitionNo::START);
    ASSERT_EQ(NodeStateNo::COMPONENT_RUNNING, sm.getCurrentState());

    sm.doTransition(NodeTransitionNo::DISCONNECT);
    ASSERT_EQ(NodeStateNo::COMPONENT_DISCONNECTED, sm.getCurrentState());
}

TEST(AbstractNodeSmTest, nodeStillRunningWhenRunningAndDisconnectFails)
{
    AbstractNodeSm sm(cb(true), cb(true), cb(false));

    sm.doTransition(NodeTransitionNo::INIT);
    sm.doTransition(NodeTransitionNo::CONNECT);
    sm.doTransition(NodeTransitionNo::START);
    ASSERT_EQ(NodeStateNo::COMPONENT_RUNNING, sm.getCurrentState());

    sm.doTransition(NodeTransitionNo::DISCONNECT);
    ASSERT_EQ(NodeStateNo::COMPONENT_RUNNING, sm.getCurrentState());
}

TEST(AbstractNodeSmTest, nodeDisconnectedWhenPausedAndDisconnectTriggered)
{
    AbstractNodeSm sm(cb(true), cb(true), cb(true), startCb(false, true));
    sm.doTransition(NodeTransitionNo::INIT);
    sm.doTransition(NodeTransitionNo::CONNECT);
    sm.doTransition(NodeTransitionNo::START);
    ASSERT_EQ(NodeStateNo::COMPONENT_PAUSED, sm.getCurrentState());

    sm.doTransition(NodeTransitionNo::DISCONNECT);
    ASSERT_EQ(NodeStateNo::COMPONENT_DISCONNECTED, sm.getCurrentState());
}

TEST(AbstractNodeSmTest, nodeStillPausedWhenPausedAndDisconnectFails)
{
    AbstractNodeSm sm(cb(true), cb(true), cb(false), startCb(false, true));

    sm.doTransition(NodeTransitionNo::INIT);
    sm.doTransition(NodeTransitionNo::CONNECT);
    sm.doTransition(NodeTransitionNo::START);
    ASSERT_EQ(NodeStateNo::COMPONENT_PAUSED, sm.getCurrentState());

    sm.doTransition(NodeTransitionNo::DISCONNECT);
    ASSERT_EQ(NodeStateNo::COMPONENT_PAUSED, sm.getCurrentState());
}

TEST(abstract_node_test, nodeStoppedWhenDisconnectedAndStopTriggered)
{
    AbstractNodeSm sm;

    sm.doTransition(NodeTransitionNo::INIT);
    sm.doTransition(NodeTransitionNo::CONNECT);
    sm.doTransition(NodeTransitionNo::DISCONNECT);
    ASSERT_EQ(NodeStateNo::COMPONENT_DISCONNECTED, sm.getCurrentState());

    sm.doTransition(NodeTransitionNo::STOP);
    ASSERT_EQ(NodeStateNo::STOPPED, sm.getCurrentState());
}

TEST(AbstractNodeSmTest, nodeStoppedWhenConfiguredAndStopTriggered)
{
    AbstractNodeSm sm;

    sm.doTransition(NodeTransitionNo::INIT);
    ASSERT_EQ(NodeStateNo::NODE_CONFIGURED, sm.getCurrentState());

    sm.doTransition(NodeTransitionNo::STOP);
    ASSERT_EQ(NodeStateNo::STOPPED, sm.getCurrentState());
}
