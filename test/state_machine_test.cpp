#include "rosen_abstract_node/state_machine.h"
#include "rosen_abstract_node/NodeState.h"

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <stdexcept> 

using namespace rosen_abstract_node;
using namespace testing;


TEST(state_machine_test, initial_state)
{
    state_machine sm(NodeState::STOPPED);
    ASSERT_EQ(NodeState::STOPPED, sm.get_current_state());
}

TEST(state_machine_test, good_transition)
{
    MockFunction<bool()> mock_callback;
    EXPECT_CALL(mock_callback, Call()).WillOnce(Return(true));
    state_machine sm(NodeState::STOPPED);
    sm.add_transition(NodeTransition::INIT,
                      {NodeState::STOPPED},
                      NodeState::NODE_CONFIGURED,
                      mock_callback.AsStdFunction());
    sm.do_transition(NodeTransition::INIT);
    ASSERT_EQ(NodeState::NODE_CONFIGURED, sm.get_current_state());
}

TEST(state_machine_test, bad_transition)
{
    MockFunction<bool()> mock_callback;
    EXPECT_CALL(mock_callback, Call()).Times(0);
    state_machine sm(NodeState::COMPONENT_CONNECTED);
    sm.add_transition(NodeTransition::INIT,
                      {NodeState::STOPPED},
                      NodeState::NODE_CONFIGURED,
                      mock_callback.AsStdFunction());
    sm.do_transition(NodeTransition::INIT);
    ASSERT_EQ(NodeState::COMPONENT_CONNECTED, sm.get_current_state());
}

TEST(state_machine_test, invalid_transition)
{
    MockFunction<bool()> mock_callback;
    EXPECT_CALL(mock_callback, Call()).Times(0);
    state_machine sm(NodeState::STOPPED);
    sm.add_transition(NodeTransition::INIT,
                      {NodeState::STOPPED},
                      NodeState::NODE_CONFIGURED,
                      mock_callback.AsStdFunction());
    EXPECT_THROW(sm.do_transition(NodeTransition::STOP), std::out_of_range);
}
