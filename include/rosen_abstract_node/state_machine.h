#ifndef ROSEN_ABSTRACT_NODE_STATE_MACHINE_H
#define ROSEN_ABSTRACT_NODE_STATE_MACHINE_H

#include <functional>
#include <vector>

#include "rosen_abstract_node/node_state_helper.h"
#include "rosen_abstract_node/node_transition_helper.h"


namespace rosen_abstract_node
{
    /**
     *
     * @brief This struct contains everything relevant for the state machine transition.
     *
     */
    struct state_machine_transition {
        std::vector<node_state_no> sources;
        node_state_no target;
        std::function<bool()> callback;
    };

    class state_machine
    {
        /**
         *
         * @brief This is a general state machine. The user can add nodes and transitions.
         *
         * @param initial_state The initial state of the state machine.
         *
         */
        public:
            state_machine(const node_state_no initial_state);
            /**
             * @brief Gets the current state the node is in.
             * 
             * @return The current state the node is in.
             */
            node_state_no get_current_state() const;

            /**
             * @brief Sets the current state of the node. This function should be used with caution.
             *        Usually you would change the node state, by performing transitions.
             */
            void set_current_state(const node_state_no current_state);

            /**
             * @brief Add a transition with the corresponding nodes and callbacks to the state machine.
             * 
             * @param node_transition The transition (directed edge) between the nodes.
             * @param sources         The source nodes of the transition. It is possible to have multiple
             *                        source nodes for a transition to a target node.
             * @param target          The target node of the transition.
             * @param callback        The callback function, which will be performed, when the transition
             *                        is called.
             */
            void add_transition(const node_transition_no node_transition, const std::vector<node_state_no>& sources, const node_state_no target, std::function<bool()> callback);

            /**
             * @brief Perform a transition from the source node to the target node and call the callback function.
             * 
             * @param transition      The transition, which should be performed.
             * 
             * @return True if transition was successful, else false.
             */
            bool do_transition(node_transition_no transition);

        private:
            node_state_no current_state;
            std::unordered_map<node_transition_no, state_machine_transition> transitions;
    };
}

#endif
