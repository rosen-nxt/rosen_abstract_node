#ifndef ROSEN_ABSTRACT_NODE_NODE_TRANSITIONS_HELPER_H
#define ROSEN_ABSTRACT_NODE_NODE_TRANSITIONS_HELPER_H

#include "rosen_abstract_node/NodeTransition.h"

#include <string>
#include <unordered_map>

namespace rosen_abstract_node
{
    using node_transition_no = unsigned char;

    namespace node_transition_helper
    {
        const std::string invalid = "INVALID";

        namespace detail
        {
            const std::unordered_map<node_transition_no, std::string> names =
            {
                { rosen_abstract_node::NodeTransition::NONE, "NONE" },
                { rosen_abstract_node::NodeTransition::INIT, "INIT" },
                { rosen_abstract_node::NodeTransition::CONNECT, "CONNECT" },
                { rosen_abstract_node::NodeTransition::DISCONNECT, "DISCONNECT" },
                { rosen_abstract_node::NodeTransition::START, "START" },
                { rosen_abstract_node::NodeTransition::PAUSE, "PAUSE" },
                { rosen_abstract_node::NodeTransition::RESUME, "RESUME" },
                { rosen_abstract_node::NodeTransition::STOP, "STOP" },
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
            return detail::names.find(transition) != detail::names.end();
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
            auto search = detail::names.find(transition);
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
