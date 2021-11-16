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
         * @param nodeName        The ROS node name of the abstract node.
         * @param nodeTransition  This is the transition, which should be performed.
         *
         * @return True if the transition is successful, otherwise False.
         * 
         */
        bool doNodeTransition(ros::NodeHandle& nh, const std::string& nodeName, const uint8_t nodeTransition);

        /**
         *
         * @brief Set the abstract node to running.
         *
         * @param nh              The ROS node handle.
         * @param nodeName        The ROS node name of the abstract node.
         *
         * @return True if the transition to running is successful, otherwise False.
         * 
         */
        bool setNodeToRunning(ros::NodeHandle& nh, const std::string& nodeName);

        /**
         *
         * @brief Wait until the abstract node is in an expected state.
         *
         * @param nh             The ROS node handle.
         * @param nodeName       The ROS node name of the abstract node.
         * @param expectedState  The expected state.
         * @param timout         After this duration the timout is triggered and this function return False.
         *
         * @return True if the abstract node is in the expected state, otherwise False.
         * 
         */
        bool waitForNodeInState(ros::NodeHandle& nh, const std::string& nodeName, const uint8_t expectedState, const ros::Duration& timeout);

        /**
         *
         * @brief Wait until the abstract node is running
         *
         * @param nh        The ROS node handle.
         * @param nodeName  The ROS node name of the abstract node.
         * @param timout    After this duration the timout is triggered and this function returns False.
         *
         * @return True if the abstract node is running, otherwise False.
         * 
         */
        bool waitForNodeRunning(ros::NodeHandle& nh, const std::string& nodeName, const ros::Duration& timeout);

        class NodeClient
        {
        /**
         *
         * @brief This class allows to perform transitions on an abstract node.
         *
         */

        private:
            ros::NodeHandle& nh;
            const std::string& nodeName;
        public:
            NodeClient(ros::NodeHandle& nh_, const std::string& nodeName_) : nh(nh_), nodeName(nodeName_) {}

            /**
             *
             * @brief Perform a node transition on the abstract node.
             *
             * @param nodeTransition This is the transition, which should be performed.
             *
             * @return True if the transition is successful, otherwise False.
             * 
             */
            bool doNodeTransition(const uint8_t nodeTransition)
            {
                return test_utils::doNodeTransition(nh, nodeName, nodeTransition);
            }

            /**
             *
             * @brief Set the abstract node to running.
             * 
             * @return True if the transition to running is successful, otherwise False.
             * 
             */
            bool setNodeToRunning()
            {
                return test_utils::setNodeToRunning(nh, nodeName);
            }

            /**
             *
             * @brief Wait until the abstract node is in an expected state.
             *
             * @param expectedState The expected state.
             * @param timout         After this duration the timout is triggered and this function return False.
             *
             * @return True if the abstract node is in the expected state, otherwise False.
             * 
             */
            bool waitForNodeInState(const uint8_t expectedState, const ros::Duration& timeout)
            {
                return test_utils::waitForNodeInState(nh, nodeName, expectedState, timeout);
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
            bool waitForNodeRunning(const ros::Duration& timeout)
            {
                return test_utils::waitForNodeRunning(nh, nodeName, timeout);
            }            
        };
    }
}

#endif