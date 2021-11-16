#include <algorithm>

#include "rosen_abstract_node/StateMachine.h"

namespace rosen_abstract_node
{
    state_machine::state_machine(const node_state_no initial_state) : current_state(initial_state){}

    node_state_no state_machine::get_current_state() const
    {
        return current_state;
    }

    void state_machine::set_current_state(const node_state_no state)
    {
        current_state = state;
    }

    void state_machine::add_transition(const node_transition_no node_transition, const std::vector<node_state_no>& sources, const node_state_no target, std::function<bool()> callback)
    {
        state_machine_transition transition {sources, target, callback};
        transitions.insert({node_transition, transition});
    }

    bool state_machine::do_transition(node_transition_no node_transition)
    {
        auto transition = transitions.at(node_transition);
        auto sources = transition.sources;

        // is current_state in sources?
        if(std::find(sources.begin(), sources.end(), current_state) != sources.end()) {
            auto success = transition.callback();
            if (success)
            {
                current_state = transition.target;
            }
            return success;
        }

        return false;
    }
}
