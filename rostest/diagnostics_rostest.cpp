#include <gtest/gtest.h>
#include <ros/ros.h>

#include <diagnostic_msgs/DiagnosticArray.h>

using namespace testing;

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "diagnostics_rostest");
    ros::NodeHandle nh("~");
    const auto test_filter = nh.param<std::string>("test_filter", "*");
    const auto test_filter_expression = "diagnostics_rostest." + test_filter;
    testing::GTEST_FLAG(filter) = test_filter_expression;
    return RUN_ALL_TESTS();
}

TEST(diagnostics_rostest, diagnostic_messages_with_correct_hardware_id_published)
{ 
    ros::NodeHandle nh("~");
    const auto full_qualified_node_name = nh.param<std::string>("full_qualified_node_name", "n.a.");

    const auto message = ros::topic::waitForMessage<diagnostic_msgs::DiagnosticArray>("/diagnostics", ros::Duration(10.0));
    ASSERT_TRUE(message != nullptr);
    ASSERT_EQ(1, message->status.size());
    const auto status_message = message->status[0];
    ASSERT_EQ(full_qualified_node_name, status_message.hardware_id);
}