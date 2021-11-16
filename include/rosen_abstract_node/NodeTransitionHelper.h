#ifndef ROSEN_ABSTRACT_NODE_NODE_TRANSITIONS_HELPER_H
#define ROSEN_ABSTRACT_NODE_NODE_TRANSITIONS_HELPER_H

#include <string>
#include <unordered_map>

namespace rosen_abstract_node
{
    enum NodeTransitionNo : unsigned char
    {
        NONE = 0,
        INIT = 1,
        CONNECT = 2,
        DISCONNECT = 3,
        START = 4,
        PAUSE = 5,
        RESUME = 6,
        STOP = 7
    };

    const std::string INVALID_TRANSITION = "INVALID";

    namespace detail
    {
        const std::unordered_map<NodeTransitionNo, std::string> TRANSITION_NAMES =
        {
            { NodeTransitionNo::NONE, "NONE" },
            { NodeTransitionNo::INIT, "INIT" },
            { NodeTransitionNo::CONNECT, "CONNECT" },
            { NodeTransitionNo::DISCONNECT, "DISCONNECT" },
            { NodeTransitionNo::START, "START" },
            { NodeTransitionNo::PAUSE, "PAUSE" },
            { NodeTransitionNo::RESUME, "RESUME" },
            { NodeTransitionNo::STOP, "STOP" },
        };
    }

    /**
     * @brief Indicates whether the passed value is a valid node transition.
     *
     * @param transition A number representing a node transition (rosen_abstract_node::NodeTransition)
     *
     * @return True if the transition is a valid node transition, else False.
     */
    inline bool isValid(const NodeTransitionNo transition)
    {
        return detail::TRANSITION_NAMES.find(transition) != detail::TRANSITION_NAMES.end();
    }

    /**
     * @brief Converts the passed node transition into a string.
     *
     * @param transition A number representing a node transition (rosen_abstract_node::NodeTransition)
     *
     * @return A string representation of the node transition or "INVALID" if the
     *         provided transition is not valid..
     */
    inline const std::string& toString(const NodeTransitionNo transition)
    {
        auto search = detail::TRANSITION_NAMES.find(transition);
        if (search != detail::TRANSITION_NAMES.end())
        {
            return search->second;
        }
        else
        {
            return INVALID_TRANSITION;
        }
    }

    namespace node_transition_helper
    {
        /**
         * @brief Indicates whether the passed value is a valid node transition.
         *
         * @param transition A number representing a node transition (rosen_abstract_node::NodeTransition)
         *
         * @return True if the transition is a valid node transition, else False.
         */
        inline bool isValid(const unsigned char transition)
        {
            return isValid(NodeTransitionNo(transition));
        }

        /**
         * @brief Converts the passed node transition into a string.
         *
         * @param transition A number representing a node transition (rosen_abstract_node::NodeTransition)
         *
         * @return A string representation of the node transition or "INVALID" if the
         *         provided transition is not valid..
         */
        inline const std::string& toString(const unsigned char transition)
        {
            return toString(NodeTransitionNo(transition));
        }
    }
}

#endif
