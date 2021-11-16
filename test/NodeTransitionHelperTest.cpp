#include "rosen_abstract_node/NodeTransition.h"
#include "rosen_abstract_node/NodeTransitionHelper.h"
#include <limits>
#include <string>
#include <gtest/gtest.h>

using namespace rosen_abstract_node;
using namespace testing;

TEST(node_transition_test, to_string_none)
{
    ASSERT_EQ(node_transition_helper::to_string(NodeTransition::NONE), "NONE");
}

TEST(node_transition_test, to_string_init)
{
    ASSERT_EQ(node_transition_helper::to_string(NodeTransition::INIT), "INIT");
}

TEST(node_transition_test, to_string_connect)
{
    ASSERT_EQ(node_transition_helper::to_string(NodeTransition::CONNECT), "CONNECT");
}

TEST(node_transition_test, to_string_disconnect)
{
    ASSERT_EQ(node_transition_helper::to_string(NodeTransition::DISCONNECT), "DISCONNECT");
}

TEST(node_transition_test, to_string_start)
{
    ASSERT_EQ(node_transition_helper::to_string(NodeTransition::START), "START");
}

TEST(node_transition_test, to_string_pause)
{
    ASSERT_EQ(node_transition_helper::to_string(NodeTransition::PAUSE), "PAUSE");
}

TEST(node_transition_test, to_string_resume)
{
    ASSERT_EQ(node_transition_helper::to_string(NodeTransition::RESUME), "RESUME");
}

TEST(node_transition_test, to_string_stop)
{
    ASSERT_EQ(node_transition_helper::to_string(NodeTransition::STOP), "STOP");
}

TEST(node_transition_test, to_string_min_unsigned_char_none)
{
    ASSERT_EQ(node_transition_helper::to_string(std::numeric_limits<unsigned char>::min()), "NONE");
}

TEST(node_transition_test, to_string_max_unsigned_char_invalid)
{
    ASSERT_EQ(node_transition_helper::to_string(std::numeric_limits<unsigned char>::max()), "INVALID");
}
