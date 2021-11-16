#ifndef ROSEN_ABSTRACT_NODE_NODE_TRANSITIONS_HELPER_H
#define ROSEN_ABSTRACT_NODE_NODE_TRANSITIONS_HELPER_H

#include <string>
#include <unordered_map>

namespace rosen_abstract_node
{
    enum node_transition_no : unsigned char
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

    const std::string invalid_transition = "INVALID";

    namespace detail
    {
        const std::unordered_map<node_transition_no, std::string> transition_names =
        {
            { node_transition_no::NONE, "NONE" },
            { node_transition_no::INIT, "INIT" },
            { node_transition_no::CONNECT, "CONNECT" },
            { node_transition_no::DISCONNECT, "DISCONNECT" },
            { node_transition_no::START, "START" },
            { node_transition_no::PAUSE, "PAUSE" },
            { node_transition_no::RESUME, "RESUME" },
            { node_transition_no::STOP, "STOP" },
        };
    }

    /**
     * @brief Indicates whether the passed value is a valid node transition.
     *
     * @param transition A number representing a node transition (rosen_abstract_node::NodeTransition)
     *
     * @return True if the transition is a valid node transition, else False.
     */
    inline bool is_valid(const node_transition_no transition)
    {
        return detail::transition_names.find(transition) != detail::transition_names.end();
    }

    /**
     * @brief Converts the passed node transition into a string.
     *
     * @param transition A number representing a node transition (rosen_abstract_node::NodeTransition)
     *
     * @return A string representation of the node transition or "INVALID" if the
     *         provided transition is not valid..
     */
    inline const std::string& to_string(const node_transition_no transition)
    {
        auto search = detail::transition_names.find(transition);
        if (search != detail::transition_names.end())
        {
            return search->second;
        }
        else
        {
            return invalid_transition;
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
        inline bool is_valid(const unsigned char transition)
        {
            return is_valid(node_transition_no(transition));
        }

        /**
         * @brief Converts the passed node transition into a string.
         *
         * @param transition A number representing a node transition (rosen_abstract_node::NodeTransition)
         *
         * @return A string representation of the node transition or "INVALID" if the
         *         provided transition is not valid..
         */
        inline const std::string& to_string(const unsigned char transition)
        {
            return to_string(node_transition_no(transition));
        }
    }
}

#endif
