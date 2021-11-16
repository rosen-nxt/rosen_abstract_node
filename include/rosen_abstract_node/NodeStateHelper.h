#ifndef ROSEN_ABSTRACT_NODE_NODE_STATE_HELPER_H
#define ROSEN_ABSTRACT_NODE_NODE_STATE_HELPER_H

#include <string>
#include <unordered_map>

namespace rosen_abstract_node
{
    enum NodeStateNo : unsigned char
    {
        STOPPED = 1,
        NODE_CONFIGURED = 2,
        COMPONENT_CONNECTED = 3,
        COMPONENT_RUNNING = 4,
        COMPONENT_PAUSED = 5,
        COMPONENT_DISCONNECTED = 6
    };

    const std::string INVALID_STATE = "INVALID";

    namespace detail
    {
        const std::unordered_map<NodeStateNo, std::string> STATE_NAMES =
        {
            { NodeStateNo::STOPPED, "STOPPED" },
            { NodeStateNo::NODE_CONFIGURED, "NODE_CONFIGURED" },
            { NodeStateNo::COMPONENT_CONNECTED, "COMPONENT_CONNECTED" },
            { NodeStateNo::COMPONENT_RUNNING, "COMPONENT_RUNNING" },
            { NodeStateNo::COMPONENT_PAUSED, "COMPONENT_PAUSED" },
            { NodeStateNo::COMPONENT_DISCONNECTED, "COMPONENT_DISCONNECTED" }
        };
    }

    /**
     * @brief Indicates whether the passed value is a valid node state.
     *
     * @param state A number representing a node state (rosen_abstract_node::NodeState)
     *
     * @return True if the state is a valid node state, else False.
     */
    inline bool isValid(const NodeStateNo state)
    {
        return detail::STATE_NAMES.find(state) != detail::STATE_NAMES.end();
    }

    /**
     * @brief Converts the passed node state into a string.
     *
     * @param state A number representing a node state (rosen_abstract_node::NodeState)
     *
     * @return A string representation of the node state or "INVALID" if the
     * provided state is not valid.
     */
    inline const std::string& toString(const NodeStateNo state)
    {
        auto search = detail::STATE_NAMES.find(state);
        if (search != detail::STATE_NAMES.end())
        {
            return search->second;
        }
        else
        {
            return INVALID_STATE;
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
        inline bool isValid(const unsigned char state)
        {
            return isValid(NodeStateNo(state));
        }

        /**
         * @brief Converts the passed node state into a string.
         *
         * @param state A number representing a node state (rosen_abstract_node::NodeState)
         *
         * @return A string representation of the node state or "INVALID" if the
         * provided state is not valid.
         */
        inline const std::string& toString(const unsigned char state)
        {
            return toString(NodeStateNo(state));
        }

    }
}

#endif
