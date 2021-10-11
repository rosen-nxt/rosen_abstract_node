#ifndef ABSTRACT_NODE_SM_H
#define ABSTRACT_NODE_SM_H

#include "rosen_abstract_node/state_machine.h"


/**
 * This class is a concrete implementation of the state machine for the abstract node.
 */

namespace rosen_abstract_node
{
    namespace detail
    {
        const std::function<bool()> EMPTY_CB = [](){return true;};
        const std::function<bool(bool&)> EMPTY_START_CB = [](bool& running){running = true; return true;};
    }

    class abstract_node_sm
    {
        public:
            /**
             * @param do_init       Optional function, which is during the INIT transition called.
             * @param do_connect    Optional function, which is during the CONNECT transition called.
             * @param do_disconnect Optional function, which is during the DISCONNECT transition called.
             * @param do_start      Optional function, which is during the START transition called.
             * @param do_pause      Optional function, which is during the PAUSE transition called.
             * @param do_resume     Optional function, which is during the RESUME transition called.
             * @param do_stop       Optional function, which is during the STOP transition called.
             */
            abstract_node_sm(std::function<bool()> do_init = detail::EMPTY_CB,
                             std::function<bool()> do_connect = detail::EMPTY_CB,
                             std::function<bool()> do_disconnect = detail::EMPTY_CB,
                             std::function<bool(bool&)> do_start = detail::EMPTY_START_CB,
                             std::function<bool()> do_pause = detail::EMPTY_CB,
                             std::function<bool()> do_resume = detail::EMPTY_CB,
                             std::function<bool()> do_stop = detail::EMPTY_CB);

            /**
             * @brief Check if the node is in a certain state.
             * 
             * @param The node state, to be checked.
             * 
             * @return Whether or not the node is in the given state.
             */
            bool is_in_node_state(const node_state_no state) const;

            /**
             * @brief Perform a transition.
             * 
             * @param The transition, which should be performed.
             * 
             * @return Whether or not the transition was successful.
             */
            bool do_transition(node_transition_no next_trans);

            /**
             * @brief Gets the current state of the node.
             * 
             * @return The current state of the node.
             */
            node_state_no get_current_state() const;

        private:
            state_machine sm;

            void init_sm();
            bool sm_start();

            const std::unordered_map<node_transition_no, std::function<bool()>> do_callbacks;
            std::function<bool(bool&)> do_start;
    };
}

#endif
