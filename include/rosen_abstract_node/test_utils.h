#ifndef ROSEN_ABSTRACT_NODE_TEST_UTILS_H
#define ROSEN_ABSTRACT_NODE_TEST_UTILS_H

#include <ros/ros.h>

namespace rosen_abstract_node
{
    namespace test_utils
    {
        /**
         *
         * @brief Perform a node transition on the abstract node.
         *
         * @param nh              The ROS node handle.
         * @param node_name       The ROS node name of the abstract node.
         * @param node_transition This is the transition, which should be performed.
         *
         * @return True if the transition is successful, otherwise False.
         * 
         */
        bool do_node_transition(ros::NodeHandle& nh, const std::string& node_name, const uint8_t node_transition);

        /**
         *
         * @brief Set the abstract node to running.
         *
         * @param nh              The ROS node handle.
         * @param node_name       The ROS node name of the abstract node.
         *
         * @return True if the transition to running is successful, otherwise False.
         * 
         */
        bool set_node_to_running(ros::NodeHandle& nh, const std::string& node_name);

        /**
         *
         * @brief Wait until the abstract node is in an expected state.
         *
         * @param nh             The ROS node handle.
         * @param node_name      The ROS node name of the abstract node.
         * @param expected_state The expected state.
         * @param timout         After this duration the timout is triggered and this function return False.
         *
         * @return True if the abstract node is in the expected state, otherwise False.
         * 
         */
        bool wait_for_node_in_state(ros::NodeHandle& nh, const std::string& node_name, const uint8_t expected_state, const ros::Duration& timeout);

        /**
         *
         * @brief Wait until the abstract node is running
         *
         * @param nh        The ROS node handle.
         * @param node_name The ROS node name of the abstract node.
         * @param timout    After this duration the timout is triggered and this function returns False.
         *
         * @return True if the abstract node is running, otherwise False.
         * 
         */
        bool wait_for_node_running(ros::NodeHandle& nh, const std::string& node_name, const ros::Duration& timeout);

        class node_client
        {
        /**
         *
         * @brief This class allows to perform transitions on an abstract node.
         *
         */

        private:
            ros::NodeHandle& nh;
            const std::string& node_name;
        public:
            node_client(ros::NodeHandle& nh_, const std::string& node_name_) : nh(nh_), node_name(node_name_) {}

            /**
             *
             * @brief Perform a node transition on the abstract node.
             *
             * @param node_transition This is the transition, which should be performed.
             *
             * @return True if the transition is successful, otherwise False.
             * 
             */
            bool do_node_transition(const uint8_t node_transition)
            {
                return test_utils::do_node_transition(nh, node_name, node_transition);
            }

            /**
             *
             * @brief Set the abstract node to running.
             * 
             * @return True if the transition to running is successful, otherwise False.
             * 
             */
            bool set_node_to_running()
            {
                return test_utils::set_node_to_running(nh, node_name);
            }

            /**
             *
             * @brief Wait until the abstract node is in an expected state.
             *
             * @param expected_state The expected state.
             * @param timout         After this duration the timout is triggered and this function return False.
             *
             * @return True if the abstract node is in the expected state, otherwise False.
             * 
             */
            bool wait_for_node_in_state(const uint8_t expected_state, const ros::Duration& timeout)
            {
                return test_utils::wait_for_node_in_state(nh, node_name, expected_state, timeout);
            }
            
            /**
             *
             * @brief Wait until the abstract node is running
             *
             * @param timout    After this duration the timout is triggered and this function return False.
             *
             * @return True if the abstract node is running, otherwise False.
             * 
             */
            bool wait_for_node_running(const ros::Duration& timeout)
            {
                return test_utils::wait_for_node_running(nh, node_name, timeout);
            }            
        };
    }
}

#endif