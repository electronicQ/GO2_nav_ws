#include <ros/ros.h>

#include <gtest/gtest.h>

TEST(RosShutdown, SpinFunctionsDoNotCreateExecutorsAfterShutdown) {
  int argc = 0;
  char **argv = nullptr;
  rclcpp::init(argc, argv);
  ros::NodeHandle node;
  ASSERT_TRUE(node.ok());
  EXPECT_NO_THROW(ros::spinOnce());
  rclcpp::shutdown();
  ASSERT_FALSE(node.ok());
  EXPECT_NO_THROW(ros::spinOnce());
  EXPECT_NO_THROW(ros::spin());
}
