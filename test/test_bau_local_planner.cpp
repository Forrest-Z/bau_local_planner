#include <gtest/gtest.h>
#include <math.h>
#include <ros/ros.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_srvs/Empty.h"
/**
 * This is a basic setup test that just sets an initial pose for the robot, as you
 * would do in RVIZ, and then checks to see if that pose correctly filters through to
 * the localisation system. It just sets up the scenario for the following tests.
 */
TEST(BAUTestHarness, testCase1) {
  ROS_INFO("Performing Test 1: Initial test scenario setup.");
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
  // wait for someone to start listening
  while (pub.getNumSubscribers() < 1) {
    loop_rate.sleep();
  }
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
  initial_pose.pose.covariance = {
      0.25, 0.0,  0.0, 0.0, 0.0, 0.0,
      0.0,  0.25, 0.0, 0.0, 0.0, 0.0,
      0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
      0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
      0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
      0.0,  0.0,  0.0, 0.0, 0.0, 0.06853892326654787};  // required for the localisation system to re-localise quicker
  // send the initial pose
  pub.publish(initial_pose);
  // sleep for a bit here, so the above can be picked up, processed and
  // filter through to the localisation system, and the localisation can
  // settle into the new pose
  // clear the costmaps, which will have old data on them
  // ros::ServiceClient client = nh_.serviceClient<std_srvs::Empty>("/move_base_node/clear_costmaps");
  // ros::service::waitForService("/move_base_node/clear_costmaps", ros::Duration(5.0));
  // std_srvs::Empty srv;
  // client.call(srv);
  // give localisation time to settle after clearing the costmap
  // now check that it worked as expected and the robot is localised
  // first subscribe to the pose estimate from the localisation system
  // then pack it into estimated_pose
  ros::Time start_time = ros::Time::now();
  ros::Duration timeout = ros::Duration(5.0);
  while (ros::ok() && ros::Time::now() - start_time < timeout) {
    ros::spinOnce();  // wait for the message to appear
  }
  // now we just compare the ground-truth initial pose we gave to the robot
  // with the estimated pose from the localisation system
  float xd = initial_pose.pose.pose.position.x - estimated_pose.pose.pose.position.x;
  float yd = initial_pose.pose.pose.position.y - estimated_pose.pose.pose.position.y;
  float td = abs(initial_pose.pose.pose.orientation.w - estimated_pose.pose.pose.orientation.w);
  float dist = sqrt(xd * xd + yd * yd);
  float dist_threshold = 0.15;
  float ang_threshold = 0.15;
  // pass the test if they're roughly close to one-another
  ASSERT_LT(dist, dist_threshold);  // in the right position
  ASSERT_LT(td, ang_threshold);     // in the right orientation
  ros::Duration(5.0).sleep();
}

/**
 * If the above test passed, the robot is now in a known position, so this next
 * test tests the actual path planner by simply moving the robot to a new pose
 * and checks to see if it is achieved within a specified time period.
 */
TEST(BAUTestHarness, testCase2) {
  ROS_INFO("Performing Test 2: Short hop.");
  ros::NodeHandle nh_;
  ros::Publisher pub;
  ros::Subscriber sub;
  ros::Rate loop_rate(10);
  pub = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  geometry_msgs::PoseWithCovarianceStamped estimated_pose;
  bool received = false;
  sub = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
      "/amcl_pose", 1, [&](const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
        estimated_pose = *msg;
        received = true;
      });
  // here we'll give the robot 10 seconds to complete this task, which should be super easy
  // wait for someone to start listening
  while (pub.getNumSubscribers() < 1) {
    loop_rate.sleep();
  }
  geometry_msgs::PoseStamped target_pose;
  target_pose.header.frame_id = "map";
  target_pose.header.stamp = ros::Time::now();
  target_pose.pose.position.x = 0.5099998712539673;
  target_pose.pose.position.y = -0.6100000143051147;
  target_pose.pose.position.z = 0;
  target_pose.pose.orientation.x = 0.0;
  target_pose.pose.orientation.y = 0.0;
  target_pose.pose.orientation.z = -0.01571748504599795;
  target_pose.pose.orientation.w = 0.9998764727024178;
  pub.publish(target_pose);
  ros::Time start_time = ros::Time::now();
  ros::Duration timeout = ros::Duration(10.0);
  while (ros::ok() && ros::Time::now() - start_time < timeout) {
    ros::spinOnce();  // wait for the message to appear
  }
  // here we do the same as the previous test -- check the distance between the
  // pose we asked the robot to assume and where it thinks it is now
  // TODO: should add a helper class to cover this kind of re-usable functionality
  float xd = target_pose.pose.position.x - estimated_pose.pose.pose.position.x;
  float yd = target_pose.pose.position.y - estimated_pose.pose.pose.position.y;
  float td = abs(target_pose.pose.orientation.w - estimated_pose.pose.pose.orientation.w);
  float dist = sqrt(xd * xd + yd * yd);
  float dist_threshold = 0.15;
  float ang_threshold = 0.15;
  // pass the test if they're roughly close to one-another
  ASSERT_LT(dist, dist_threshold);  // in the right position
  ASSERT_LT(td, ang_threshold);     // in the right orientation
}

/**
TEST(BAUTestHarness, testCase3) {
  ROS_INFO("Performing Test 3: Long hop.");
  ros::NodeHandle nh_;
  ros::Publisher pub;

  EXPECT_EQ(1, 1);
}

TEST(BAUTestHarness, testCase4) {
  ROS_INFO("Performing Test 3: Stuck check.");
  ros::NodeHandle nh_;
  ros::Publisher pub;

  EXPECT_EQ(1, 1);
}
 */
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "bau_local_planner_tests");
  ros::NodeHandle nh_;
  return RUN_ALL_TESTS();
}
