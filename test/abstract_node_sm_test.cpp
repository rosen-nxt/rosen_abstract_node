#include "rosen_abstract_node/abstract_node_sm.h"
#include "rosen_abstract_node/node_state_helper.h"
#include "rosen_abstract_node/node_transition_helper.h"

#include <gtest/gtest.h>

using namespace rosen_abstract_node;
using namespace testing;


std::function<bool()> cb(const bool return_value)
{
    return [return_value](){return return_value;};
}

std::function<bool(bool&)> start_cb(const bool running_arg, const bool return_value)
{
    return [running_arg, return_value](bool& running){running = running_arg; return return_value;};
}

TEST(abstract_node_sm_test, initial_state)
{
    abstract_node_sm sm;
    ASSERT_EQ(node_state_no::STOPPED, sm.get_current_state());
}

TEST(abstract_node_sm_test, node_configured_after_init_if_successful)
{
    abstract_node_sm sm;

    sm.do_transition(node_transition_no::INIT);
    ASSERT_EQ(node_state_no::NODE_CONFIGURED, sm.get_current_state());
}

TEST(abstract_node_sm_test, node_still_stopped_after_init_if_failed)
{
    abstract_node_sm sm(cb(false));

    sm.do_transition(node_transition_no::INIT);
    ASSERT_EQ(node_state_no::STOPPED, sm.get_current_state());
}

TEST(abstract_node_sm_test, node_connected_after_connect_if_successful)
{
    abstract_node_sm sm;

    sm.do_transition(node_transition_no::INIT);
    ASSERT_EQ(node_state_no::NODE_CONFIGURED, sm.get_current_state());

    sm.do_transition(node_transition_no::CONNECT);
    ASSERT_EQ(node_state_no::COMPONENT_CONNECTED, sm.get_current_state());
}

TEST(abstract_node_sm_test, still_configured_after_connect_if_failed)
{
    abstract_node_sm sm(cb(true), cb(false));

    sm.do_transition(node_transition_no::INIT);
    ASSERT_EQ(node_state_no::NODE_CONFIGURED, sm.get_current_state());

    sm.do_transition(node_transition_no::CONNECT);
    ASSERT_EQ(node_state_no::NODE_CONFIGURED, sm.get_current_state());
}

TEST(abstract_node_sm_test, node_running_after_start_if_successful_and_not_paused)
{
    abstract_node_sm sm;

    sm.do_transition(node_transition_no::INIT);
    sm.do_transition(node_transition_no::CONNECT);
    ASSERT_EQ(node_state_no::COMPONENT_CONNECTED, sm.get_current_state());

    sm.do_transition(node_transition_no::START);
    ASSERT_EQ(node_state_no::COMPONENT_RUNNING, sm.get_current_state());
}

TEST(abstract_node_sm_test, node_paused_after_start_if_successful_and_paused)
{
    abstract_node_sm sm(cb(true), cb(true), cb(true), start_cb(false, true));

    sm.do_transition(node_transition_no::INIT);
    sm.do_transition(node_transition_no::CONNECT);
    ASSERT_EQ(node_state_no::COMPONENT_CONNECTED, sm.get_current_state());

    sm.do_transition(node_transition_no::START);
    ASSERT_EQ(node_state_no::COMPONENT_PAUSED, sm.get_current_state());
}

TEST(abstract_node_sm_test, node_paused_when_running_and_pause_triggered)
{
    abstract_node_sm sm;

    sm.do_transition(node_transition_no::INIT);
    sm.do_transition(node_transition_no::CONNECT);
    sm.do_transition(node_transition_no::START);
    ASSERT_EQ(node_state_no::COMPONENT_RUNNING, sm.get_current_state());

    sm.do_transition(node_transition_no::PAUSE);
    ASSERT_EQ(node_state_no::COMPONENT_PAUSED, sm.get_current_state());
}

TEST(abstract_node_sm_test, node_still_running_when_running_and_pause_failed)
{
    abstract_node_sm sm(cb(true), cb(true), cb(true), start_cb(true, true), cb(false));

    sm.do_transition(node_transition_no::INIT);
    sm.do_transition(node_transition_no::CONNECT);
    sm.do_transition(node_transition_no::START);
    ASSERT_EQ(node_state_no::COMPONENT_RUNNING, sm.get_current_state());

    sm.do_transition(node_transition_no::PAUSE);
    ASSERT_EQ(node_state_no::COMPONENT_RUNNING, sm.get_current_state());
}

TEST(abstract_node_sm_test, node_running_when_paused_and_resume_triggered)
{
    abstract_node_sm sm(cb(true), cb(true), cb(true), start_cb(false, true));

    sm.do_transition(node_transition_no::INIT);
    sm.do_transition(node_transition_no::CONNECT);
    sm.do_transition(node_transition_no::START);
    ASSERT_EQ(node_state_no::COMPONENT_PAUSED, sm.get_current_state());

    sm.do_transition(node_transition_no::RESUME);
    ASSERT_EQ(node_state_no::COMPONENT_RUNNING, sm.get_current_state());
}

TEST(abstract_node_sm_test, node_still_paused_when_paused_and_resume_failed)
{
    abstract_node_sm sm(cb(true), cb(true), cb(true), start_cb(false, true), cb(true), cb(false));

    sm.do_transition(node_transition_no::INIT);
    sm.do_transition(node_transition_no::CONNECT);
    sm.do_transition(node_transition_no::START);
    ASSERT_EQ(node_state_no::COMPONENT_PAUSED, sm.get_current_state());

    sm.do_transition(node_transition_no::RESUME);
    ASSERT_EQ(node_state_no::COMPONENT_PAUSED, sm.get_current_state());
}

TEST(abstract_node_sm_test, node_disconnected_when_running_and_disconnect_triggered)
{
    abstract_node_sm sm;

    sm.do_transition(node_transition_no::INIT);
    sm.do_transition(node_transition_no::CONNECT);
    sm.do_transition(node_transition_no::START);
    ASSERT_EQ(node_state_no::COMPONENT_RUNNING, sm.get_current_state());

    sm.do_transition(node_transition_no::DISCONNECT);
    ASSERT_EQ(node_state_no::COMPONENT_DISCONNECTED, sm.get_current_state());
}

TEST(abstract_node_sm_test, node_still_running_when_running_and_disconnect_fails)
{
    abstract_node_sm sm(cb(true), cb(true), cb(false));

    sm.do_transition(node_transition_no::INIT);
    sm.do_transition(node_transition_no::CONNECT);
    sm.do_transition(node_transition_no::START);
    ASSERT_EQ(node_state_no::COMPONENT_RUNNING, sm.get_current_state());

    sm.do_transition(node_transition_no::DISCONNECT);
    ASSERT_EQ(node_state_no::COMPONENT_RUNNING, sm.get_current_state());
}

TEST(abstract_node_sm_test, node_disconnected_when_paused_and_disconnect_triggered)
{
    abstract_node_sm sm(cb(true), cb(true), cb(true), start_cb(false, true));
    sm.do_transition(node_transition_no::INIT);
    sm.do_transition(node_transition_no::CONNECT);
    sm.do_transition(node_transition_no::START);
    ASSERT_EQ(node_state_no::COMPONENT_PAUSED, sm.get_current_state());

    sm.do_transition(node_transition_no::DISCONNECT);
    ASSERT_EQ(node_state_no::COMPONENT_DISCONNECTED, sm.get_current_state());
}

TEST(abstract_node_sm_test, node_still_paused_when_paused_and_disconnect_fails)
{
    abstract_node_sm sm(cb(true), cb(true), cb(false), start_cb(false, true));

    sm.do_transition(node_transition_no::INIT);
    sm.do_transition(node_transition_no::CONNECT);
    sm.do_transition(node_transition_no::START);
    ASSERT_EQ(node_state_no::COMPONENT_PAUSED, sm.get_current_state());

    sm.do_transition(node_transition_no::DISCONNECT);
    ASSERT_EQ(node_state_no::COMPONENT_PAUSED, sm.get_current_state());
}

TEST(abstract_node_test, node_stopped_when_disconnected_and_stop_triggered)
{
    abstract_node_sm sm;

    sm.do_transition(node_transition_no::INIT);
    sm.do_transition(node_transition_no::CONNECT);
    sm.do_transition(node_transition_no::DISCONNECT);
    ASSERT_EQ(node_state_no::COMPONENT_DISCONNECTED, sm.get_current_state());

    sm.do_transition(node_transition_no::STOP);
    ASSERT_EQ(node_state_no::STOPPED, sm.get_current_state());
}

TEST(abstract_node_sm_test, node_stopped_when_configured_and_stop_triggered)
{
    abstract_node_sm sm;

    sm.do_transition(node_transition_no::INIT);
    ASSERT_EQ(node_state_no::NODE_CONFIGURED, sm.get_current_state());

    sm.do_transition(node_transition_no::STOP);
    ASSERT_EQ(node_state_no::STOPPED, sm.get_current_state());
}
