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
    : sm(NodeState::STOPPED),
      do_callbacks{
        {NodeTransition::INIT, do_init},
        {NodeTransition::CONNECT, do_connect},
        {NodeTransition::DISCONNECT, do_disconnect},
        {NodeTransition::PAUSE, do_pause},
        {NodeTransition::RESUME, do_resume},
        {NodeTransition::STOP, do_stop},
                   },
      do_start(do_start)
    {
        init_sm();
    }

    void abstract_node_sm::init_sm()
    {
        sm.add_transition(NodeTransition::INIT,
                          {NodeState::STOPPED},
                          NodeState::NODE_CONFIGURED,
                          do_callbacks.at(NodeTransition::INIT));
        sm.add_transition(NodeTransition::CONNECT,
                          {NodeState::NODE_CONFIGURED, NodeState::COMPONENT_DISCONNECTED},
                          NodeState::COMPONENT_CONNECTED,
                          do_callbacks.at(NodeTransition::CONNECT));
        sm.add_transition(NodeTransition::DISCONNECT,
                          {NodeState::COMPONENT_PAUSED, NodeState::COMPONENT_RUNNING, NodeState::COMPONENT_CONNECTED},
                          NodeState::COMPONENT_DISCONNECTED,
                          do_callbacks.at(NodeTransition::DISCONNECT));
        sm.add_transition(NodeTransition::PAUSE,
                          {NodeState::COMPONENT_RUNNING},
                          NodeState::COMPONENT_PAUSED,
                          do_callbacks.at(NodeTransition::PAUSE));
        sm.add_transition(NodeTransition::RESUME,
                          {NodeState::COMPONENT_PAUSED},
                          NodeState::COMPONENT_RUNNING,
                          do_callbacks.at(NodeTransition::RESUME));
        sm.add_transition(NodeTransition::STOP,
                          {NodeState::NODE_CONFIGURED, NodeState::COMPONENT_DISCONNECTED},
                          NodeState::STOPPED,
                          do_callbacks.at(NodeTransition::STOP));
    }

    bool abstract_node_sm::sm_start()
    {
        if (sm.get_current_state() == NodeState::COMPONENT_CONNECTED)
        {
            bool running = false;
            if (do_start(running))
            {
                sm.set_current_state(running ?    NodeState::COMPONENT_RUNNING : NodeState::COMPONENT_PAUSED);
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
        if (next_trans == NodeTransition::START)
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
