#include <string>
#include <sstream>

#include "ros/ros.h"

#include "rosen_abstract_node/TestUtils.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "execute_abstract_node_transitions");

    ros::NodeHandle nh("~");
    auto actionNamespace = nh.param<std::string>("namespace", "no_namespace");
    ROS_INFO("execute_abstract_node_transitions: Action namespace is %s", actionNamespace.c_str());
    
    auto transitionsString = nh.param<std::string>("transitions", "");
    auto retryFirstTransition = nh.param<bool>("retry_first_transition", false);
    if (retryFirstTransition)
    {
        ROS_INFO("execute_abstract_node_transitions: Will retry first transition as long as it fails.");
    }

    std::vector<int> transitions;    
    {
        std::istringstream transitionsStringstream(transitionsString);
        int transition;
        while (transitionsStringstream >> transition)
        {
            transitions.push_back(transition);
        }
    }

    if (transitions.empty())
    {
        ROS_ERROR("execute_abstract_node_transitions: No transitions passed to parameter '~/transitions'.");
    }

    bool first = true;
    for (auto& transition: transitions)
    {
        ROS_INFO("execute_abstract_node_transitions: Execute transition: %d", transition);
        auto success = rosen_abstract_node::test_utils::doNodeTransition(nh, actionNamespace, transition);
        while (first && !success && retryFirstTransition) 
        {
            ROS_WARN("execute_abstract_node_transitions: Execute transition %d failed, retrying...", transition);
            ros::Duration(1).sleep();
            success = rosen_abstract_node::test_utils::doNodeTransition(nh, actionNamespace, transition);
        }

        if (!success)
        {
            ROS_ERROR("execute_abstract_node_transitions: Execute transition %d FAILED", transition);
        }

        first = false;
    }

    return 0;
 }