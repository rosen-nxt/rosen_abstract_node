#include "rosen_abstract_node/NodeTransition.h"
#include "rosen_abstract_node/NodeState.h"
#include "rosen_abstract_node/rosen_abstract_node.h"
#include <memory>
#include <string>
#include <gtest/gtest.h>
#include <gmock/gmock.h>

using namespace rosen_abstract_node;
using namespace testing;

namespace node_test
{
    class mock_abstract_node : public rosen_abstract_node::rosen_abstract_node
    {
        public:
            mock_abstract_node() :
                rosen_abstract_node(std::string("node_name"), nullptr, nullptr) {}
            virtual ~mock_abstract_node() = default;

            void do_loop_once()
            {
                do_transition(nullptr);
                do_step();
            }

            MOCK_METHOD0(do_init, bool());
            MOCK_METHOD0(do_stop, bool());
            MOCK_METHOD0(do_connect, bool());
            MOCK_METHOD0(do_disconnect, bool());
            MOCK_METHOD0(do_configure_component, void());
            MOCK_METHOD0(do_pause, bool());
            MOCK_METHOD0(do_resume, bool());
            MOCK_METHOD1(do_start, bool(bool& running));
            
            MOCK_METHOD0(do_step, void());
    };
}

TEST(abstract_node_test, node_configured_after_init_if_successful)
{
    StrictMock<node_test::mock_abstract_node> node;

    EXPECT_CALL(node, do_init()).Times(1).WillOnce(Return(true));
    EXPECT_CALL(node, do_step()).Times(2);
    ASSERT_EQ(NodeState::STOPPED, node.get_current_state());

    node.do_loop_once();
    ASSERT_EQ(NodeState::STOPPED, node.get_current_state());

    node.initiate_transition(NodeTransition::INIT);
    node.do_loop_once();
    ASSERT_EQ(NodeState::NODE_CONFIGURED, node.get_current_state());
}

TEST(abstract_node_test, node_still_stopped_after_init_if_failed_transition_has_no_retry)
{
    StrictMock<node_test::mock_abstract_node> node;

    EXPECT_CALL(node, do_init()).Times(1).WillOnce(Return(false));
    EXPECT_CALL(node, do_step()).Times(4);
    ASSERT_EQ(NodeState::STOPPED, node.get_current_state());

    node.do_loop_once();
    ASSERT_EQ(NodeState::STOPPED, node.get_current_state());

    node.initiate_transition(NodeTransition::INIT);
    node.do_loop_once();
    ASSERT_EQ(NodeState::STOPPED, node.get_current_state());
    
    node.do_loop_once();
    ASSERT_EQ(NodeState::STOPPED, node.get_current_state());
    
    node.do_loop_once();
    ASSERT_EQ(NodeState::STOPPED, node.get_current_state());
}

TEST(abstract_node_test, node_still_stopped_after_init_if_failed)
{
    StrictMock<node_test::mock_abstract_node> node;

    EXPECT_CALL(node, do_init()).Times(1).WillOnce(Return(false));
    EXPECT_CALL(node, do_step()).Times(2);
    ASSERT_EQ(NodeState::STOPPED, node.get_current_state());

    node.do_loop_once();
    ASSERT_EQ(NodeState::STOPPED, node.get_current_state());

    node.initiate_transition(NodeTransition::INIT);
    node.do_loop_once();
    ASSERT_EQ(NodeState::STOPPED, node.get_current_state());
}

TEST(abstract_node_test, node_connected_after_connect_if_successful)
{
    StrictMock<node_test::mock_abstract_node> node;

    EXPECT_CALL(node, do_init()).Times(1).WillOnce(Return(true));
    EXPECT_CALL(node, do_connect()).Times(1).WillOnce(Return(true));
    EXPECT_CALL(node, do_step()).Times(2);
    
    node.initiate_transition(NodeTransition::INIT);
    node.do_loop_once();
    ASSERT_EQ(NodeState::NODE_CONFIGURED, node.get_current_state());

    node.initiate_transition(NodeTransition::CONNECT);
    node.do_loop_once();
    ASSERT_EQ(NodeState::COMPONENT_CONNECTED, node.get_current_state());
}

TEST(abstract_node_test, still_configured_after_connect_if_failed)
{
    StrictMock<node_test::mock_abstract_node> node;

    EXPECT_CALL(node, do_init()).Times(1).WillOnce(Return(true));
    EXPECT_CALL(node, do_connect()).Times(1).WillOnce(Return(false));
    EXPECT_CALL(node, do_step()).Times(2);
    
    node.initiate_transition(NodeTransition::INIT);
    node.do_loop_once();
    ASSERT_EQ(NodeState::NODE_CONFIGURED, node.get_current_state());

    node.initiate_transition(NodeTransition::CONNECT);
    node.do_loop_once();
    ASSERT_EQ(NodeState::NODE_CONFIGURED, node.get_current_state());
}

TEST(abstract_node_test, node_running_after_start_if_successful_and_not_paused)
{
    StrictMock<node_test::mock_abstract_node> node;

    EXPECT_CALL(node, do_init()).Times(1).WillOnce(Return(true));
    EXPECT_CALL(node, do_connect()).Times(1).WillOnce(Return(true));
    EXPECT_CALL(node, do_start(_)).Times(1).WillOnce(DoAll(SetArgReferee<0>(true), Return(true)));
    EXPECT_CALL(node, do_step()).Times(3);

    node.initiate_transition(NodeTransition::INIT);
    node.do_loop_once();
    node.initiate_transition(NodeTransition::CONNECT);
    node.do_loop_once();	
    ASSERT_EQ(NodeState::COMPONENT_CONNECTED, node.get_current_state());

    node.initiate_transition(NodeTransition::START);
    node.do_loop_once();	
    ASSERT_EQ(NodeState::COMPONENT_RUNNING, node.get_current_state());
}

TEST(abstract_node_test, node_paused_after_start_if_successful_and_paused)
{
    StrictMock<node_test::mock_abstract_node> node;

    EXPECT_CALL(node, do_init()).Times(1).WillOnce(Return(true));
    EXPECT_CALL(node, do_connect()).Times(1).WillOnce(Return(true));
    EXPECT_CALL(node, do_start(_)).Times(1).WillOnce(DoAll(SetArgReferee<0>(false), Return(true)));
    EXPECT_CALL(node, do_step()).Times(3);

    node.initiate_transition(NodeTransition::INIT);
    node.do_loop_once();
    node.initiate_transition(NodeTransition::CONNECT);
    node.do_loop_once();	
    ASSERT_EQ(NodeState::COMPONENT_CONNECTED, node.get_current_state());

    node.initiate_transition(NodeTransition::START);
    node.do_loop_once();	
    ASSERT_EQ(NodeState::COMPONENT_PAUSED, node.get_current_state());
}

TEST(abstract_node_test, node_paused_when_running_and_pause_triggered)
{
    StrictMock<node_test::mock_abstract_node> node;

    EXPECT_CALL(node, do_init()).Times(1).WillOnce(Return(true));
    EXPECT_CALL(node, do_connect()).Times(1).WillOnce(Return(true));
    EXPECT_CALL(node, do_start(_)).Times(1).WillOnce(DoAll(SetArgReferee<0>(true), Return(true)));
    EXPECT_CALL(node, do_step()).Times(4);
    EXPECT_CALL(node, do_pause()).Times(1).WillOnce(Return(true));

    node.initiate_transition(NodeTransition::INIT);
    node.do_loop_once();
    node.initiate_transition(NodeTransition::CONNECT);
    node.do_loop_once();
    node.initiate_transition(NodeTransition::START);
    node.do_loop_once();	
    ASSERT_EQ(NodeState::COMPONENT_RUNNING, node.get_current_state());

    node.initiate_transition(NodeTransition::PAUSE);
    node.do_loop_once();	
    ASSERT_EQ(NodeState::COMPONENT_PAUSED, node.get_current_state());
}

TEST(abstract_node_test, node_still_running_when_running_and_pause_failed)
{
    StrictMock<node_test::mock_abstract_node> node;

    EXPECT_CALL(node, do_init()).Times(1).WillOnce(Return(true));
    EXPECT_CALL(node, do_connect()).Times(1).WillOnce(Return(true));
    EXPECT_CALL(node, do_start(_)).Times(1).WillOnce(DoAll(SetArgReferee<0>(true), Return(true)));
    EXPECT_CALL(node, do_step()).Times(4);
    EXPECT_CALL(node, do_pause()).Times(1).WillOnce(Return(false));

    node.initiate_transition(NodeTransition::INIT);
    node.do_loop_once();
    node.initiate_transition(NodeTransition::CONNECT);
    node.do_loop_once();
    node.initiate_transition(NodeTransition::START);
    node.do_loop_once();	
    ASSERT_EQ(NodeState::COMPONENT_RUNNING, node.get_current_state());

    node.initiate_transition(NodeTransition::PAUSE);
    node.do_loop_once();	
    ASSERT_EQ(NodeState::COMPONENT_RUNNING, node.get_current_state());
}

TEST(abstract_node_test, node_running_when_paused_and_resume_triggered)
{
    StrictMock<node_test::mock_abstract_node> node;

    EXPECT_CALL(node, do_init()).Times(1).WillOnce(Return(true));
    EXPECT_CALL(node, do_connect()).Times(1).WillOnce(Return(true));
    EXPECT_CALL(node, do_start(_)).Times(1).WillOnce(DoAll(SetArgReferee<0>(false), Return(true)));
    EXPECT_CALL(node, do_step()).Times(4);
    EXPECT_CALL(node, do_resume()).Times(1).WillOnce(Return(true));

    node.initiate_transition(NodeTransition::INIT);
    node.do_loop_once();
    node.initiate_transition(NodeTransition::CONNECT);
    node.do_loop_once();
    node.initiate_transition(NodeTransition::START);
    node.do_loop_once();	
    ASSERT_EQ(NodeState::COMPONENT_PAUSED, node.get_current_state());

    node.initiate_transition(NodeTransition::RESUME);
    node.do_loop_once();	
    ASSERT_EQ(NodeState::COMPONENT_RUNNING, node.get_current_state());
}

TEST(abstract_node_test, node_still_paused_when_paused_and_resume_failed)
{
    StrictMock<node_test::mock_abstract_node> node;

    EXPECT_CALL(node, do_init()).Times(1).WillOnce(Return(true));
    EXPECT_CALL(node, do_connect()).Times(1).WillOnce(Return(true));
    EXPECT_CALL(node, do_start(_)).Times(1).WillOnce(DoAll(SetArgReferee<0>(false), Return(true)));
    EXPECT_CALL(node, do_step()).Times(4);
    EXPECT_CALL(node, do_resume()).Times(1).WillOnce(Return(false));

    node.initiate_transition(NodeTransition::INIT);
    node.do_loop_once();
    node.initiate_transition(NodeTransition::CONNECT);
    node.do_loop_once();
    node.initiate_transition(NodeTransition::START);
    node.do_loop_once();	
    ASSERT_EQ(NodeState::COMPONENT_PAUSED, node.get_current_state());

    node.initiate_transition(NodeTransition::RESUME);
    node.do_loop_once();	
    ASSERT_EQ(NodeState::COMPONENT_PAUSED, node.get_current_state());
}

TEST(abstract_node_test, node_disconnected_when_running_and_disconnect_triggered)
{
    StrictMock<node_test::mock_abstract_node> node;

    EXPECT_CALL(node, do_init()).Times(1).WillOnce(Return(true));
    EXPECT_CALL(node, do_connect()).Times(1).WillOnce(Return(true));
    EXPECT_CALL(node, do_start(_)).Times(1).WillOnce(DoAll(SetArgReferee<0>(true), Return(true)));
    EXPECT_CALL(node, do_step()).Times(4);
    EXPECT_CALL(node, do_disconnect()).Times(1).WillOnce(Return(true));

    node.initiate_transition(NodeTransition::INIT);
    node.do_loop_once();
    node.initiate_transition(NodeTransition::CONNECT);
    node.do_loop_once();
    node.initiate_transition(NodeTransition::START);
    node.do_loop_once();	
    ASSERT_EQ(NodeState::COMPONENT_RUNNING, node.get_current_state());

    node.initiate_transition(NodeTransition::DISCONNECT);
    node.do_loop_once();	
    ASSERT_EQ(NodeState::COMPONENT_DISCONNECTED, node.get_current_state());
}

TEST(abstract_node_test, node_still_running_when_running_and_disconnect_fails)
{
    StrictMock<node_test::mock_abstract_node> node;

    EXPECT_CALL(node, do_init()).Times(1).WillOnce(Return(true));
    EXPECT_CALL(node, do_connect()).Times(1).WillOnce(Return(true));
    EXPECT_CALL(node, do_start(_)).Times(1).WillOnce(DoAll(SetArgReferee<0>(true), Return(true)));
    EXPECT_CALL(node, do_step()).Times(4);
    EXPECT_CALL(node, do_disconnect()).Times(1).WillOnce(Return(false));

    node.initiate_transition(NodeTransition::INIT);
    node.do_loop_once();
    node.initiate_transition(NodeTransition::CONNECT);
    node.do_loop_once();
    node.initiate_transition(NodeTransition::START);
    node.do_loop_once();	
    ASSERT_EQ(NodeState::COMPONENT_RUNNING, node.get_current_state());

    node.initiate_transition(NodeTransition::DISCONNECT);
    node.do_loop_once();	
    ASSERT_EQ(NodeState::COMPONENT_RUNNING, node.get_current_state());
}

TEST(abstract_node_test, node_disconnected_when_paused_and_disconnect_triggered)
{
    StrictMock<node_test::mock_abstract_node> node;

    EXPECT_CALL(node, do_init()).Times(1).WillOnce(Return(true));
    EXPECT_CALL(node, do_connect()).Times(1).WillOnce(Return(true));
    EXPECT_CALL(node, do_start(_)).Times(1).WillOnce(DoAll(SetArgReferee<0>(false), Return(true)));
    EXPECT_CALL(node, do_step()).Times(4);
    EXPECT_CALL(node, do_disconnect()).Times(1).WillOnce(Return(true));

    node.initiate_transition(NodeTransition::INIT);
    node.do_loop_once();
    node.initiate_transition(NodeTransition::CONNECT);
    node.do_loop_once();
    node.initiate_transition(NodeTransition::START);
    node.do_loop_once();	
    ASSERT_EQ(NodeState::COMPONENT_PAUSED, node.get_current_state());

    node.initiate_transition(NodeTransition::DISCONNECT);
    node.do_loop_once();	
    ASSERT_EQ(NodeState::COMPONENT_DISCONNECTED, node.get_current_state());
}

TEST(abstract_node_test, node_still_paused_when_paused_and_disconnect_fails)
{
    StrictMock<node_test::mock_abstract_node> node;

    EXPECT_CALL(node, do_init()).Times(1).WillOnce(Return(true));
    EXPECT_CALL(node, do_connect()).Times(1).WillOnce(Return(true));
    EXPECT_CALL(node, do_start(_)).Times(1).WillOnce(DoAll(SetArgReferee<0>(false), Return(true)));
    EXPECT_CALL(node, do_step()).Times(4);
    EXPECT_CALL(node, do_disconnect()).Times(1).WillOnce(Return(false));

    node.initiate_transition(NodeTransition::INIT);
    node.do_loop_once();
    node.initiate_transition(NodeTransition::CONNECT);
    node.do_loop_once();
    node.initiate_transition(NodeTransition::START);
    node.do_loop_once();	
    ASSERT_EQ(NodeState::COMPONENT_PAUSED, node.get_current_state());

    node.initiate_transition(NodeTransition::DISCONNECT);
    node.do_loop_once();	
    ASSERT_EQ(NodeState::COMPONENT_PAUSED, node.get_current_state());
}

TEST(abstract_node_test, node_stopped_when_disconnected_and_stop_triggered)
{
    StrictMock<node_test::mock_abstract_node> node;

    EXPECT_CALL(node, do_init()).Times(1).WillOnce(testing::Return(true));
    EXPECT_CALL(node, do_connect()).Times(1).WillOnce(testing::Return(true));
    EXPECT_CALL(node, do_disconnect()).Times(1).WillOnce(testing::Return(true));
    EXPECT_CALL(node, do_stop()).Times(1).WillOnce(testing::Return(true));
    EXPECT_CALL(node, do_step()).Times(4);
    
    node.initiate_transition(NodeTransition::INIT);
    node.do_loop_once();
    node.initiate_transition(NodeTransition::CONNECT);
    node.do_loop_once();
    node.initiate_transition(NodeTransition::DISCONNECT);
    node.do_loop_once();
    ASSERT_EQ(NodeState::COMPONENT_DISCONNECTED, node.get_current_state());

    node.initiate_transition(NodeTransition::STOP);
    node.do_loop_once();
    ASSERT_EQ(NodeState::STOPPED, node.get_current_state());
}

TEST(abstract_node_test, node_stopped_when_configured_and_stop_triggered)
{
    StrictMock<node_test::mock_abstract_node> node;

    EXPECT_CALL(node, do_init()).Times(1).WillOnce(Return(true));
    EXPECT_CALL(node, do_stop()).Times(1).WillOnce(Return(true));
    EXPECT_CALL(node, do_step()).Times(2);

    node.initiate_transition(NodeTransition::INIT);
    node.do_loop_once();
    ASSERT_EQ(NodeState::NODE_CONFIGURED, node.get_current_state());

    node.initiate_transition(NodeTransition::STOP);
    node.do_loop_once();
    ASSERT_EQ(NodeState::STOPPED, node.get_current_state());
}
