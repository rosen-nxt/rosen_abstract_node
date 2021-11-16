#ifndef ABSTRACT_NODE_SM_H
#define ABSTRACT_NODE_SM_H

#include "rosen_abstract_node/StateMachine.h"


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

    class AbstractNodeSm
    {
        public:
            /**
             * @param doInit       Optional function, which is during the INIT transition called.
             * @param doConnect    Optional function, which is during the CONNECT transition called.
             * @param doDisconnect Optional function, which is during the DISCONNECT transition called.
             * @param doStart      Optional function, which is during the START transition called.
             * @param doPause      Optional function, which is during the PAUSE transition called.
             * @param doResume     Optional function, which is during the RESUME transition called.
             * @param doStop       Optional function, which is during the STOP transition called.
             */
            AbstractNodeSm(std::function<bool()> doInit = detail::EMPTY_CB,
                            std::function<bool()> doConnect = detail::EMPTY_CB,
                            std::function<bool()> doDisconnect = detail::EMPTY_CB,
                            std::function<bool(bool&)> doStart = detail::EMPTY_START_CB,
                            std::function<bool()> doPause = detail::EMPTY_CB,
                            std::function<bool()> doResume = detail::EMPTY_CB,
                            std::function<bool()> doStop = detail::EMPTY_CB);

            /**
             * @brief Check if the node is in a certain state.
             * 
             * @param The node state, to be checked.
             * 
             * @return Whether or not the node is in the given state.
             */
            bool isInNodeState(const NodeStateNo state) const;

            /**
             * @brief Perform a transition.
             * 
             * @param The transition, which should be performed.
             * 
             * @return Whether or not the transition was successful.
             */
            bool doTransition(NodeTransitionNo nextTrans);

            /**
             * @brief Gets the current state of the node.
             * 
             * @return The current state of the node.
             */
            NodeStateNo getCurrentState() const;

        private:
            StateMachine sm;

            void initSm();
            bool smStart();

            const std::unordered_map<NodeTransitionNo, std::function<bool()>> doCallbacks;
            std::function<bool(bool&)> doStart;
    };
}

#endif
