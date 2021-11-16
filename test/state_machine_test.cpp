#include "rosen_abstract_node/state_machine.h"
#include "rosen_abstract_node/node_state_helper.h"
#include "rosen_abstract_node/node_transition_helper.h"

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <stdexcept> 

using namespace rosen_abstract_node;
using namespace testing;


TEST(state_machine_test, initial_state)
{
    state_machine sm(node_state_no::STOPPED);
    ASSERT_EQ(node_state_no::STOPPED, sm.get_current_state());
}

TEST(state_machine_test, good_transition)
{
    MockFunction<bool()> mock_callback;
    EXPECT_CALL(mock_callback, Call()).WillOnce(Return(true));
    state_machine sm(node_state_no::STOPPED);
    sm.add_transition(node_transition_no::INIT,
                      {node_state_no::STOPPED},
                      node_state_no::NODE_CONFIGURED,
                      mock_callback.AsStdFunction());
    sm.do_transition(node_transition_no::INIT);
    ASSERT_EQ(node_state_no::NODE_CONFIGURED, sm.get_current_state());
}

TEST(state_machine_test, bad_transition)
{
    MockFunction<bool()> mock_callback;
    EXPECT_CALL(mock_callback, Call()).Times(0);
    state_machine sm(node_state_no::COMPONENT_CONNECTED);
    sm.add_transition(node_transition_no::INIT,
                      {node_state_no::STOPPED},
                      node_state_no::NODE_CONFIGURED,
                      mock_callback.AsStdFunction());
    sm.do_transition(node_transition_no::INIT);
    ASSERT_EQ(node_state_no::COMPONENT_CONNECTED, sm.get_current_state());
}

TEST(state_machine_test, invalid_transition)
{
    MockFunction<bool()> mock_callback;
    EXPECT_CALL(mock_callback, Call()).Times(0);
    state_machine sm(node_state_no::STOPPED);
    sm.add_transition(node_transition_no::INIT,
                      {node_state_no::STOPPED},
                      node_state_no::NODE_CONFIGURED,
                      mock_callback.AsStdFunction());
    EXPECT_THROW(sm.do_transition(node_transition_no::STOP), std::out_of_range);
}
