#include <algorithm>

#include "rosen_abstract_node/StateMachine.h"

namespace rosen_abstract_node
{
    StateMachine::StateMachine(const NodeStateNo initialState) : currentState(initialState){}

    NodeStateNo StateMachine::getCurrentState() const
    {
        return currentState;
    }

    void StateMachine::setCurrentState(const NodeStateNo state)
    {
        currentState = state;
    }

    void StateMachine::addTransition(const NodeTransitionNo nodeTransition, const std::vector<NodeStateNo>& sources, const NodeStateNo target, std::function<bool()> callback)
    {
        StateMachineTransition transition {sources, target, callback};
        transitions.insert({nodeTransition, transition});
    }

    bool StateMachine::doTransition(NodeTransitionNo nodeTransition)
    {
        auto transition = transitions.at(nodeTransition);
        auto sources = transition.sources;

        // is currentState in sources?
        if(std::find(sources.begin(), sources.end(), currentState) != sources.end()) {
            auto success = transition.callback();
            if (success)
            {
                currentState = transition.target;
            }
            return success;
        }

        return false;
    }
}
