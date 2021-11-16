#ifndef ROSEN_ABSTRACT_NODE_NODE_STATE_HELPER_H
#define ROSEN_ABSTRACT_NODE_NODE_STATE_HELPER_H

#include "rosen_abstract_node/NodeState.h"

#include <string>
#include <unordered_map>

namespace rosen_abstract_node
{
    enum node_state_no : unsigned char
    {
        STOPPED = 1,
        NODE_CONFIGURED = 2,
        COMPONENT_CONNECTED = 3,
        COMPONENT_RUNNING = 4,
        COMPONENT_PAUSED = 5,
        COMPONENT_DISCONNECTED = 6
    };

    const std::string invalid_state = "INVALID";

    namespace detail
    {
        const std::unordered_map<node_state_no, std::string> state_names =
        {
            { node_state_no::STOPPED, "STOPPED" },
            { node_state_no::NODE_CONFIGURED, "NODE_CONFIGURED" },
            { node_state_no::COMPONENT_CONNECTED, "COMPONENT_CONNECTED" },
            { node_state_no::COMPONENT_RUNNING, "COMPONENT_RUNNING" },
            { node_state_no::COMPONENT_PAUSED, "COMPONENT_PAUSED" },
            { node_state_no::COMPONENT_DISCONNECTED, "COMPONENT_DISCONNECTED" }
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
        return detail::state_names.find(state) != detail::state_names.end();
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
        auto search = detail::state_names.find(state);
        if (search != detail::state_names.end())
        {
            return search->second;
        }
        else
        {
            return invalid_state;
        }
    }

    namespace node_state_helper
    {
        /**
         * @brief Indicates whether the passed value is a valid node state.
         *
         * @param state A number representing a node state (rosen_abstract_node::NodeState)
         *
         * @return True if the state is a valid node state, else False.
         */
        inline bool is_valid(const unsigned char state)
        {
            return is_valid(node_state_no(state));
        }

        /**
         * @brief Converts the passed node state into a string.
         *
         * @param state A number representing a node state (rosen_abstract_node::NodeState)
         *
         * @return A string representation of the node state or "INVALID" if the
         * provided state is not valid.
         */
        inline const std::string& to_string(const unsigned char state)
        {
            return to_string(node_state_no(state));
        }

    }
}

#endif
