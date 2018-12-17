/**
 *  MIT License
 *
 *  Copyright (c) 2018 Saimouli Katragadda, Saurav Kumar
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without
 *  limitation the rights to use, copy, modify, merge, publish, distribute,
 *  sublicense, and/or sell copies of the Software, and to permit persons to
 *  whom the Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <gtest/gtest.h>
#include <frontier_exploration_turtlebot/CollisionDetector.h>
#include <frontier_exploration_turtlebot/PathPlanning.h>

/**
 *@file         PathPlanningTest.cpp
 *@author       Saimouli Katragadda
 *@author       Saurav Kumar
 *@copyright    MIT License
 *@brief        Tests the PathPlanning class methods
 */

// global variable for testing
float linX = 0.0, angZ;

/**
 * @brief      global function to store the twist msg value to global float variabls
 * @param      const geometry_msgs::Twist msg
 * @return     void
 */
void testCallback(const geometry_msgs::Twist msg) {
  linX = msg.linear.x;
  angZ = msg.angular.z;
}

/**
 * @brief      Tests for initialization
 * @param      PathPlanningTest             gtest framework
 * @param      InitializationErrorTest      Name of the test
 */
TEST(PathPlanningTest, InitializationErrorTest) {
  ros::NodeHandle nh;
  EXPECT_NO_FATAL_FAILURE(PathPlanning pathPlanner);
}

/**
 * @brief      Tests spiralPathGenerator method of the class PathPlanning
 * @param      PathPlanningTest             gtest framework
 * @param      spiralPathGeneratorTest      Name of the test
 * @return     none
 */
TEST(PathPlanningTest, spiralPathGeneratorTest) {
  // created NodeHandle nh
  ros::NodeHandle nh;
  // creating Publisher testpub to publish laser msg to sca topic
  ros::Publisher testPub = nh.advertise<sensor_msgs::LaserScan>("/scan", 50);
  //  creating  a laserData msg tpe
  sensor_msgs::LaserScan laserData;
  laserData.angle_min = -0.52;
  laserData.angle_max = 0.52;
  laserData.angle_increment = 0.0016;
  laserData.time_increment = 0.0;
  laserData.range_min = 0.44;
  laserData.range_max = 3.0;
  laserData.ranges.resize(50);
  laserData.intensities.resize(50);

  // initialize all the range of laserData to zero
  for (auto& i : laserData.ranges) {
    i = 0.0;
  }

  PathPlanning planner;
  // creating Subscriber testsub  subscribing to scan topic
  // and calling lasercallback function of CollisionDetector class
  ros::Subscriber testSub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 50,
                                                               testCallback);

  int counter = 0;

  while (ros::ok()) {
    // publish the laserData
    testPub.publish(laserData);
    // activate the spiral generator method
    planner.spiralPathGenerator();
    // break the loop after three iterations
    if (counter == 3) {
      break;
    }
    ros::spinOnce();
    counter++;
  }
  // check to see if the linear velocity is what we expect it to be for
  // the laser data
  EXPECT_NEAR(0.2, linX, 0.1);
  EXPECT_EQ(0.0, angZ);
}

/**
 * @brief      Tests linearPathGenerator method of the class PathPlanning
 * @param      PathPlanningTest             gtest framework
 * @param      linearPathGeneratorTest      Name of the test
 * @return     none
 */
TEST(PathPlanningTest, linearPathGeneratorTest) {
  ros::NodeHandle nh;
  // create a publisher to publish the laserscan topic
  ros::Publisher testPub = nh.advertise<sensor_msgs::LaserScan>("/scan", 50);
  //  create a laserData msg for testing purpose
  sensor_msgs::LaserScan laserData;
  laserData.angle_min = -0.52;
  laserData.angle_max = 0.52;
  laserData.angle_increment = 0.0016;
  laserData.time_increment = 0.0;
  laserData.range_min = 0.44;
  laserData.range_max = 3.0;
  laserData.ranges.resize(50);
  laserData.intensities.resize(50);

  // fill range vector to zeros
  for (auto& i : laserData.ranges) {
    i = 0.0;
  }

  // initializing planner object
  PathPlanning planner;
  // creating Subscriber testsub  subscribing to cmd_vel topic
  // and calling lasercallback function of CollisionDetector class
  ros::Subscriber testSub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 50,
                                                               testCallback);

  int counter = 0;

  while (ros::ok()) {
    // publish the test laser data
    testPub.publish(laserData);
    planner.linearPathGenerator();

    // break the loop after three iterations
    if (counter == 3) {
      break;
    }
    ros::spinOnce();
    counter++;
  }
  // check to see if the linear velocity is what we expect it to be for
  // the laser data
  EXPECT_NEAR(0.2, linX, 0.1);
  EXPECT_EQ(0.0, angZ);
}
