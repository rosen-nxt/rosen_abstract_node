#include "rosen_abstract_node/NodeTransition.h"
#include "rosen_abstract_node/NodeTransitionHelper.h"
#include <limits>
#include <string>
#include <gtest/gtest.h>

using namespace rosen_abstract_node;
using namespace testing;

TEST(NodeTransitionTest, toStringNone)
{
    ASSERT_EQ(node_transition_helper::toString(NodeTransition::NONE), "NONE");
}

TEST(NodeTransitionTest, toStringInit)
{
    ASSERT_EQ(node_transition_helper::toString(NodeTransition::INIT), "INIT");
}

TEST(NodeTransitionTest, toStringConnect)
{
    ASSERT_EQ(node_transition_helper::toString(NodeTransition::CONNECT), "CONNECT");
}

TEST(NodeTransitionTest, toStringDisconnect)
{
    ASSERT_EQ(node_transition_helper::toString(NodeTransition::DISCONNECT), "DISCONNECT");
}

TEST(NodeTransitionTest, toStringStart)
{
    ASSERT_EQ(node_transition_helper::toString(NodeTransition::START), "START");
}

TEST(NodeTransitionTest, toStringPause)
{
    ASSERT_EQ(node_transition_helper::toString(NodeTransition::PAUSE), "PAUSE");
}

TEST(NodeTransitionTest, toStringResume)
{
    ASSERT_EQ(node_transition_helper::toString(NodeTransition::RESUME), "RESUME");
}

TEST(NodeTransitionTest, toStringStop)
{
    ASSERT_EQ(node_transition_helper::toString(NodeTransition::STOP), "STOP");
}

TEST(NodeTransitionTest, toStringMinUnsignedCharNone)
{
    ASSERT_EQ(node_transition_helper::toString(std::numeric_limits<unsigned char>::min()), "NONE");
}

TEST(NodeTransitionTest, toStringMaxUnsignedCharInvalid)
{
    ASSERT_EQ(node_transition_helper::toString(std::numeric_limits<unsigned char>::max()), "INVALID");
}
