#include <gtest/gtest.h>
#include <ros/ros.h>

#include <diagnostic_msgs/DiagnosticArray.h>

using namespace testing;

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "diagnostics_rostest");
    ros::NodeHandle nh("~");
    const auto testFilter = nh.param<std::string>("test_filter", "*");
    const auto testFilterExpression = "DiagnosticsRostest." + testFilter;
    testing::GTEST_FLAG(filter) = testFilterExpression;
    return RUN_ALL_TESTS();
}

TEST(DiagnosticsRostest, diagnosticMessagesWithCorrectHardwareIdPublished)
{ 
    ros::NodeHandle nh("~");
    const auto fullQualifiedNodeName = nh.param<std::string>("full_qualified_node_name", "n.a.");

    const auto message = ros::topic::waitForMessage<diagnostic_msgs::DiagnosticArray>("/diagnostics", ros::Duration(10.0));
    ASSERT_TRUE(message != nullptr);
    ASSERT_EQ(1, message->status.size());
    const auto statusMessage = message->status[0];
    ASSERT_EQ(fullQualifiedNodeName, statusMessage.hardware_id);
}