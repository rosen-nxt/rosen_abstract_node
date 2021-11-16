#include <algorithm>

#include "rosen_abstract_node/abstract_node_sm.h"

namespace rosen_abstract_node
{
    abstract_node_sm::abstract_node_sm(std::function<bool()> do_init,
                                       std::function<bool()> do_connect,
                                       std::function<bool()> do_disconnect,
                                       std::function<bool(bool&)> do_start,
                                       std::function<bool()> do_pause,
                                       std::function<bool()> do_resume,
                                       std::function<bool()> do_stop)
    : sm(node_state_no::STOPPED),
      do_callbacks{
        {node_transition_no::INIT, do_init},
        {node_transition_no::CONNECT, do_connect},
        {node_transition_no::DISCONNECT, do_disconnect},
        {node_transition_no::PAUSE, do_pause},
        {node_transition_no::RESUME, do_resume},
        {node_transition_no::STOP, do_stop},
                   },
      do_start(do_start)
    {
        init_sm();
    }

    void abstract_node_sm::init_sm()
    {
        sm.add_transition(node_transition_no::INIT,
                          {node_state_no::STOPPED},
                          node_state_no::NODE_CONFIGURED,
                          do_callbacks.at(node_transition_no::INIT));
        sm.add_transition(node_transition_no::CONNECT,
                          {node_state_no::NODE_CONFIGURED, node_state_no::COMPONENT_DISCONNECTED},
                          node_state_no::COMPONENT_CONNECTED,
                          do_callbacks.at(node_transition_no::CONNECT));
        sm.add_transition(node_transition_no::DISCONNECT,
                          {node_state_no::COMPONENT_PAUSED, node_state_no::COMPONENT_RUNNING, node_state_no::COMPONENT_CONNECTED},
                          node_state_no::COMPONENT_DISCONNECTED,
                          do_callbacks.at(node_transition_no::DISCONNECT));
        sm.add_transition(node_transition_no::PAUSE,
                          {node_state_no::COMPONENT_RUNNING},
                          node_state_no::COMPONENT_PAUSED,
                          do_callbacks.at(node_transition_no::PAUSE));
        sm.add_transition(node_transition_no::RESUME,
                          {node_state_no::COMPONENT_PAUSED},
                          node_state_no::COMPONENT_RUNNING,
                          do_callbacks.at(node_transition_no::RESUME));
        sm.add_transition(node_transition_no::STOP,
                          {node_state_no::NODE_CONFIGURED, node_state_no::COMPONENT_DISCONNECTED},
                          node_state_no::STOPPED,
                          do_callbacks.at(node_transition_no::STOP));
    }

    bool abstract_node_sm::sm_start()
    {
        if (sm.get_current_state() == node_state_no::COMPONENT_CONNECTED)
        {
            bool running = false;
            if (do_start(running))
            {
                sm.set_current_state(running ?    node_state_no::COMPONENT_RUNNING : node_state_no::COMPONENT_PAUSED);
                return true;
            }
        }
        return false;
    }

    bool abstract_node_sm::is_in_node_state(const node_state_no state) const
    {
        return sm.get_current_state() == state;
    }

    bool abstract_node_sm::do_transition(node_transition_no next_trans)
    {
        // The start transition is an edge case, because it can result in different states, running or paused,
        // which depends on running parameter. So the start transition cannot be added to the state machine
        // and has to be implemented outside of it.
        if (next_trans == node_transition_no::START)
        {
            return sm_start();
        }
        else
        {
            return sm.do_transition(next_trans);
        }
    }

    node_state_no abstract_node_sm::get_current_state() const
    {
        return sm.get_current_state();
    }
}
