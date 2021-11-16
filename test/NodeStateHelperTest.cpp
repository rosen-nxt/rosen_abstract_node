#include "rosen_abstract_node/NodeState.h"
#include "rosen_abstract_node/NodeStateHelper.h"
#include <limits>
#include <string>
#include <gtest/gtest.h>

using namespace rosen_abstract_node;
using namespace testing;

TEST(NodeStateTest, toStringUninitialized)
{
    ASSERT_EQ(node_state_helper::toString(NodeState::STOPPED), "STOPPED");
}

TEST(NodeStateTest, toStringNodeConfigured)
{
    ASSERT_EQ(node_state_helper::toString(NodeState::NODE_CONFIGURED), "NODE_CONFIGURED");
}

TEST(NodeStateTest, toStringComponentConnected)
{
    ASSERT_EQ(node_state_helper::toString(NodeState::COMPONENT_CONNECTED), "COMPONENT_CONNECTED");
}

TEST(NodeStateTest, toStringComponentRunning)
{
    ASSERT_EQ(node_state_helper::toString(NodeState::COMPONENT_RUNNING), "COMPONENT_RUNNING");
}

TEST(NodeStateTest, toStringComponentPaused)
{
    ASSERT_EQ(node_state_helper::toString(NodeState::COMPONENT_PAUSED), "COMPONENT_PAUSED");
}

TEST(NodeStateTest, toStringComponentDisconnected)
{
    ASSERT_EQ(node_state_helper::toString(NodeState::COMPONENT_DISCONNECTED), "COMPONENT_DISCONNECTED");
}

TEST(NodeStateTest, toStringMinUnsignedCharInvalid)
{
    ASSERT_EQ(node_state_helper::toString(std::numeric_limits<unsigned char>::min()), "INVALID");
}

TEST(NodeStateTest, toStringMaxUnsignedCharInvalid)
{
    ASSERT_EQ(node_state_helper::toString(std::numeric_limits<unsigned char>::max()), "INVALID");
}
