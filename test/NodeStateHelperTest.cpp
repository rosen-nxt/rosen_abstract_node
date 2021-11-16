#include "rosen_abstract_node/NodeState.h"
#include "rosen_abstract_node/NodeStateHelper.h"
#include <limits>
#include <string>
#include <gtest/gtest.h>

using namespace rosen_abstract_node;
using namespace testing;

TEST(node_state_test, to_string_uninitialized)
{
    ASSERT_EQ(node_state_helper::to_string(NodeState::STOPPED), "STOPPED");
}

TEST(node_state_test, to_string_node_configured)
{
    ASSERT_EQ(node_state_helper::to_string(NodeState::NODE_CONFIGURED), "NODE_CONFIGURED");
}

TEST(node_state_test, to_string_component_connected)
{
    ASSERT_EQ(node_state_helper::to_string(NodeState::COMPONENT_CONNECTED), "COMPONENT_CONNECTED");
}

TEST(node_state_test, to_string_component_running)
{
    ASSERT_EQ(node_state_helper::to_string(NodeState::COMPONENT_RUNNING), "COMPONENT_RUNNING");
}

TEST(node_state_test, to_string_component_paused)
{
    ASSERT_EQ(node_state_helper::to_string(NodeState::COMPONENT_PAUSED), "COMPONENT_PAUSED");
}

TEST(node_state_test, to_string_component_disconnected)
{
    ASSERT_EQ(node_state_helper::to_string(NodeState::COMPONENT_DISCONNECTED), "COMPONENT_DISCONNECTED");
}

TEST(node_state_test, to_string_min_unsigned_char_invalid)
{
    ASSERT_EQ(node_state_helper::to_string(std::numeric_limits<unsigned char>::min()), "INVALID");
}

TEST(node_state_test, to_string_max_unsigned_char_invalid)
{
    ASSERT_EQ(node_state_helper::to_string(std::numeric_limits<unsigned char>::max()), "INVALID");
}
