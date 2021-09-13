#ifndef ROSEN_ABSTRACT_NODE_NODE_STATE_HELPER_H
#define ROSEN_ABSTRACT_NODE_NODE_STATE_HELPER_H

#include "rosen_abstract_node/NodeState.h"

#include <string>
#include <unordered_map>

namespace rosen_abstract_node
{
    using node_state_no = unsigned char;

    namespace node_state_helper
    {
        const std::string invalid = "INVALID";

        namespace detail
        {
            const std::unordered_map<node_state_no, std::string> names = 
            {
                { rosen_abstract_node::NodeState::STOPPED, "STOPPED" },
                { rosen_abstract_node::NodeState::NODE_CONFIGURED, "NODE_CONFIGURED" },
                { rosen_abstract_node::NodeState::COMPONENT_CONNECTED, "COMPONENT_CONNECTED" },
                { rosen_abstract_node::NodeState::COMPONENT_RUNNING, "COMPONENT_RUNNING" },
                { rosen_abstract_node::NodeState::COMPONENT_PAUSED, "COMPONENT_PAUSED" },
                { rosen_abstract_node::NodeState::COMPONENT_DISCONNECTED, "COMPONENT_DISCONNECTED" }
            };
        }

        /**
         * @brief Indicates whether the passed value is a valid node state.
         * 
         * @param state A number representing a node state (rosen_abstract_node::NodeState)
         * 
         * @return True if the state is a valid node state, else False.
         */
        inline bool is_valid(const node_state_no state)
        {
            return detail::names.find(state) != detail::names.end();
        }

        /**
         * @brief Converts the passed node state into a string.
         * 
         * @param state A number representing a node state (rosen_abstract_node::NodeState)
         * 
         * @return A string representation of the node state or "INVALID" if the
         * provided state is not valid.
         */
        inline const std::string& to_string(const node_state_no state)
        {
            auto search = detail::names.find(state);
            if (search != detail::names.end())
            {
                return search->second;
            }
            else
            {
                return invalid;
            }
        }
    }
}

#endif
