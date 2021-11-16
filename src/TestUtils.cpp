#include "rosen_abstract_node/TestUtils.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "rosen_abstract_node/NodeState.h"
#include "rosen_abstract_node/NodeStateInfo.h"
#include "rosen_abstract_node/NodeTransition.h"
#include "rosen_abstract_node/StateTransitionAction.h"

namespace rosen_abstract_node
{
    namespace test_utils
    {
        bool do_node_transition(ros::NodeHandle& nh, const std::string& node_name, const uint8_t node_transition)
        {
            actionlib::SimpleActionClient<rosen_abstract_node::StateTransitionAction> action_client(nh, node_name + "/state_transition_action", true);
            action_client.waitForServer();

            rosen_abstract_node::StateTransitionGoal goal;
            goal.transition = node_transition;
            return action_client.sendGoalAndWait(goal) == actionlib::SimpleClientGoalState::SUCCEEDED;
        }

        bool set_node_to_running(ros::NodeHandle& nh, const std::string& node_name)
        {
            auto succ = test_utils::do_node_transition(nh, node_name, rosen_abstract_node::NodeTransition::INIT);
            succ &= test_utils::do_node_transition(nh, node_name, rosen_abstract_node::NodeTransition::CONNECT);
            succ &= test_utils::do_node_transition(nh, node_name, rosen_abstract_node::NodeTransition::START);
            return succ;
        }

        bool wait_for_node_in_state(ros::NodeHandle& nh, const std::string& node_name, const uint8_t expected_state, const ros::Duration& timeout)
        {
            auto timeout_time = ros::Time::now() + timeout;
            while (ros::Time::now() < timeout_time)
            {
                auto node_state_info = ros::topic::waitForMessage<rosen_abstract_node::NodeStateInfo>(node_name + "/current_state", nh, timeout);
                if (node_state_info == nullptr)
                {
                    continue;
                }

                if (node_state_info->current_state == expected_state)
                {
                    return true;
                }
            }

            return false;
        }

        bool wait_for_node_running(ros::NodeHandle& nh, const std::string& node_name, const ros::Duration& timeout)
        {
            return wait_for_node_in_state(nh, node_name, rosen_abstract_node::NodeState::COMPONENT_RUNNING, timeout);
        }
    }
}