#include <gtest/gtest.h>
#include <math.h>
#include <ros/ros.h>

#include "geometry_msgs/PoseWithCovarianceStamped.h"

/**
 * This is a basic setup test that just sets an initial pose for the robot, as you
 * would do in RVIZ, and then checks to see if that pose correctly filters through to
 * the localisation system. It just sets up the scenario for the following tests.
 */
TEST(BAUTestHarness, testCase1) {
  ROS_INFO("Performing initial test scenario setup.");
  ros::NodeHandle nh_;
  ros::Publisher pub;
  ros::Subscriber sub;
  pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
  geometry_msgs::PoseWithCovarianceStamped estimated_pose;
  bool received = false;
  sub = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
      "/amcl_pose", 1, [&](const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
        estimated_pose = *msg;
        received = true;
      });
  ros::Rate loop_rate(10);
  // here we're just going to publish the initial pose of the robot in the test
  // environment, which is the first thing we'd do in rviz anyhow using the intial pose tool
  ROS_INFO("Setting initial pose.");
  geometry_msgs::PoseWithCovarianceStamped initial_pose;
  initial_pose.header.frame_id = "map";
  initial_pose.header.stamp = ros::Time::now();
  initial_pose.pose.pose.position.x = -1.97000;  // co-ordinates copied from rviz tool
  initial_pose.pose.pose.position.y = -0.51999;  // bottom-right of the pen, facing north
  initial_pose.pose.pose.position.z = 0;
  initial_pose.pose.pose.orientation.x = 0.0;
  initial_pose.pose.pose.orientation.y = 0.0;
  initial_pose.pose.pose.orientation.z = -0.00961;
  initial_pose.pose.pose.orientation.w = 0.999953;
  // wait for someone to start listening
  while (pub.getNumSubscribers() < 1) {
    loop_rate.sleep();
  }
  // send the initial pose
  pub.publish(initial_pose);
  // sleep for a second here, so the above can be picked up, processed and
  // filter through to the localisation system
  ros::Duration(2.0).sleep();
  // now check that it worked as expected and the robot is localised
  // first subscribe to the pose estimate from the localisation system
  // then pack it into estimated_pose
  while (ros::ok() && !received) {
    ros::spinOnce();  // wait for the message to appear
  }
  // now we just compare the ground-truth initial pose we gave to the robot
  // with the estimated pose from the localisation system
  float xd = initial_pose.pose.pose.position.x - estimated_pose.pose.pose.position.x;
  float yd = initial_pose.pose.pose.position.y - estimated_pose.pose.pose.position.y;
  float td = abs(initial_pose.pose.pose.orientation.w - estimated_pose.pose.pose.orientation.w);
  float dist = sqrt(xd * xd + yd * yd);
  float dist_threshold = 0.1;
  float ang_threshold = 0.1;
  // pass the test if they're roughly close to one-another
  ASSERT_LT(dist, dist_threshold);
  ASSERT_LT(td, ang_threshold);
}

TEST(BAUTestHarness, testCase2) {
  ROS_INFO("Running test 2");
  ros::NodeHandle nh_;
  ros::Publisher pub;
  pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 0);
  EXPECT_EQ(1, 1);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "bau_local_planner_tests");
  ros::NodeHandle nh_;
  return RUN_ALL_TESTS();
}
