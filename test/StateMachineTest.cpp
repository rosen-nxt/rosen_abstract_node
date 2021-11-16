#include "rosen_abstract_node/StateMachine.h"
#include "rosen_abstract_node/NodeStateHelper.h"
#include "rosen_abstract_node/NodeTransitionHelper.h"

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <stdexcept> 

using namespace rosen_abstract_node;
using namespace testing;


TEST(StateMachineTest, initial_state)
{
    StateMachine sm(NodeStateNo::STOPPED);
    ASSERT_EQ(NodeStateNo::STOPPED, sm.getCurrentState());
}

TEST(StateMachineTest, good_transition)
{
    MockFunction<bool()> mockCallback;
    EXPECT_CALL(mockCallback, Call()).WillOnce(Return(true));
    StateMachine sm(NodeStateNo::STOPPED);
    sm.addTransition(NodeTransitionNo::INIT,
                    {NodeStateNo::STOPPED},
                    NodeStateNo::NODE_CONFIGURED,
                    mockCallback.AsStdFunction());
    sm.doTransition(NodeTransitionNo::INIT);
    ASSERT_EQ(NodeStateNo::NODE_CONFIGURED, sm.getCurrentState());
}

TEST(StateMachineTest, badTransition)
{
    MockFunction<bool()> mockCallback;
    EXPECT_CALL(mockCallback, Call()).Times(0);
    StateMachine sm(NodeStateNo::COMPONENT_CONNECTED);
    sm.addTransition(NodeTransitionNo::INIT,
                     {NodeStateNo::STOPPED},
                     NodeStateNo::NODE_CONFIGURED,
                     mockCallback.AsStdFunction());
    sm.doTransition(NodeTransitionNo::INIT);
    ASSERT_EQ(NodeStateNo::COMPONENT_CONNECTED, sm.getCurrentState());
}

TEST(StateMachineTest, INVALID_TRANSITION)
{
    MockFunction<bool()> mockCallback;
    EXPECT_CALL(mockCallback, Call()).Times(0);
    StateMachine sm(NodeStateNo::STOPPED);
    sm.addTransition(NodeTransitionNo::INIT,
                     {NodeStateNo::STOPPED},
                     NodeStateNo::NODE_CONFIGURED,
                     mockCallback.AsStdFunction());
    EXPECT_THROW(sm.doTransition(NodeTransitionNo::STOP), std::out_of_range);
}
