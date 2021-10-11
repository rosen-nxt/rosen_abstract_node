#include <string>
#include <sstream>

#include "ros/ros.h"

#include "rosen_abstract_node/test_utils.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "execute_abstract_node_transitions");

    ros::NodeHandle nh("~");
    auto action_namespace = nh.param<std::string>("namespace", "no_namespace");
    ROS_INFO("execute_abstract_node_transitions: Action namespace is %s", action_namespace.c_str());
    
    auto transitions_string = nh.param<std::string>("transitions", "");
    auto retry_first_transition = nh.param<bool>("retry_first_transition", false);
    if (retry_first_transition)
    {
        ROS_INFO("execute_abstract_node_transitions: Will retry first transition as long as it fails.");
    }

    std::vector<int> transitions;    
    {
        std::istringstream transitions_stringstream(transitions_string);
        int transition;
        while (transitions_stringstream >> transition)
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
        auto success = rosen_abstract_node::test_utils::do_node_transition(nh, action_namespace, transition);
        while (first && !success && retry_first_transition) 
        {
            ROS_WARN("execute_abstract_node_transitions: Execute transition %d failed, retrying...", transition);
            ros::Duration(1).sleep();
            success = rosen_abstract_node::test_utils::do_node_transition(nh, action_namespace, transition);
        }

        if (!success)
        {
            ROS_ERROR("execute_abstract_node_transitions: Execute transition %d FAILED", transition);
        }

        first = false;
    }

    return 0;
 }