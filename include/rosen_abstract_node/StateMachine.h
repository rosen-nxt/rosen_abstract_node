#ifndef ROSEN_ABSTRACT_NODE_STATE_MACHINE_H
#define ROSEN_ABSTRACT_NODE_STATE_MACHINE_H

#include <functional>
#include <vector>

#include "rosen_abstract_node/NodeStateHelper.h"
#include "rosen_abstract_node/NodeTransitionHelper.h"


namespace rosen_abstract_node
{
    /**
     *
     * @brief This struct contains everything relevant for the state machine transition.
     *
     */
    struct StateMachineTransition {
        std::vector<NodeStateNo> sources;
        NodeStateNo target;
        std::function<bool()> callback;
    };

    class StateMachine
    {
        /**
         *
         * @brief This is a general state machine. The user can add nodes and transitions.
         *
         * @param initialState The initial state of the state machine.
         *
         */
        public:
            StateMachine(const NodeStateNo initialState);
            /**
             * @brief Gets the current state the node is in.
             * 
             * @return The current state the node is in.
             */
            NodeStateNo getCurrentState() const;

            /**
             * @brief Sets the current state of the node. This function should be used with caution.
             *        Usually you would change the node state, by performing transitions.
             */
            void setCurrentState(const NodeStateNo currentState);

            /**
             * @brief Add a transition with the corresponding nodes and callbacks to the state machine.
             * 
             * @param nodeTransition  The transition (directed edge) between the nodes.
             * @param sources         The source nodes of the transition. It is possible to have multiple
             *                        source nodes for a transition to a target node.
             * @param target          The target node of the transition.
             * @param callback        The callback function, which will be performed, when the transition
             *                        is called.
             */
            void addTransition(const NodeTransitionNo nodeTransition, const std::vector<NodeStateNo>& sources, const NodeStateNo target, std::function<bool()> callback);

            /**
             * @brief Perform a transition from the source node to the target node and call the callback function.
             * 
             * @param transition      The transition, which should be performed.
             * 
             * @return True if transition was successful, else false.
             */
            bool doTransition(NodeTransitionNo transition);

        private:
            NodeStateNo currentState;
            std::unordered_map<NodeTransitionNo, StateMachineTransition> transitions;
    };
}

#endif
