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
        bool doNodeTransition(ros::NodeHandle& nh, const std::string& nodeName, const uint8_t nodeTransition)
        {
            actionlib::SimpleActionClient<rosen_abstract_node::StateTransitionAction> action_client(nh, nodeName + "/state_transition_action", true);
            action_client.waitForServer();

            rosen_abstract_node::StateTransitionGoal goal;
            goal.transition = nodeTransition;
            return action_client.sendGoalAndWait(goal) == actionlib::SimpleClientGoalState::SUCCEEDED;
        }

        bool setNodeToRunning(ros::NodeHandle& nh, const std::string& nodeName)
        {
            auto succ = test_utils::doNodeTransition(nh, nodeName, rosen_abstract_node::NodeTransition::INIT);
            succ &= test_utils::doNodeTransition(nh, nodeName, rosen_abstract_node::NodeTransition::CONNECT);
            succ &= test_utils::doNodeTransition(nh, nodeName, rosen_abstract_node::NodeTransition::START);
            return succ;
        }

        bool waitForNodeInState(ros::NodeHandle& nh, const std::string& nodeName, const uint8_t expectedState, const ros::Duration& timeout)
        {
            auto timeout_time = ros::Time::now() + timeout;
            while (ros::Time::now() < timeout_time)
            {
                auto nodeStateInfo = ros::topic::waitForMessage<rosen_abstract_node::NodeStateInfo>(nodeName + "/current_state", nh, timeout);
                if (nodeStateInfo == nullptr)
                {
                    continue;
                }

                if (nodeStateInfo->current_state == expectedState)
                {
                    return true;
                }
            }

            return false;
        }

        bool waitForNodeRunning(ros::NodeHandle& nh, const std::string& nodeName, const ros::Duration& timeout)
        {
            return waitForNodeInState(nh, nodeName, rosen_abstract_node::NodeState::COMPONENT_RUNNING, timeout);
        }
    }
}