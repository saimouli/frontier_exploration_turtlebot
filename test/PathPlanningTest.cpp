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

float linX = 0.0, angZ;
void testCallback(const geometry_msgs::Twist msg) {
  linX = msg.linear.x;
  angZ = msg.angular.z;
}

TEST(PathPlanningTest, InitializationErrorTest) {
  ros::NodeHandle nh;
  EXPECT_NO_FATAL_FAILURE(PathPlanning pathPlanner);
}

/*
 TEST(PathPlanningTest, PathPlanningPublisherTest) {
 ros::NodeHandle nh;
 CollisionDetector cdetect;
 PathPlanning pathPlanner;
 pathPlanner.PathGenerator();
 auto sub = nh.subscribe("/cmd_vel", 50, &CollisionDetector::laserCallback,
 &cdetect);
 std::cout << sub.getNumPublishers();

 EXPECT_EQ(sub.getNumPublishers(), 1);
 }*/

TEST(PathPlanningTest, spiralPathGeneratorTest) {
  ros::NodeHandle nh;

  ros::Publisher testPub = nh.advertise<sensor_msgs::LaserScan>("/scan", 50);

  sensor_msgs::LaserScan laserData;
  laserData.angle_min = -0.52;
  laserData.angle_max = 0.52;
  laserData.angle_increment = 0.0016;
  laserData.time_increment = 0.0;
  laserData.range_min = 0.44;
  laserData.range_max = 3.0;
  laserData.ranges.resize(50);
  laserData.intensities.resize(50);

  for (auto& i : laserData.ranges) {
    i = 0.0;
  }

  PathPlanning planner;

  ros::Subscriber testSub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 50,
                                                               testCallback);

  int counter = 0;

  while (ros::ok()) {
    testPub.publish(laserData);
    planner.spiralPathGenerator();

    if (counter == 3) {
      break;
    }
    ros::spinOnce();
    counter++;
  }
  EXPECT_NEAR(0.2, linX, 0.1);
  EXPECT_EQ(0.0, angZ);
}

TEST(PathPlanningTest, linearPathGeneratorTest) {
  ros::NodeHandle nh;

  ros::Publisher testPub = nh.advertise<sensor_msgs::LaserScan>("/scan", 50);

  sensor_msgs::LaserScan laserData;
  laserData.angle_min = -0.52;
  laserData.angle_max = 0.52;
  laserData.angle_increment = 0.0016;
  laserData.time_increment = 0.0;
  laserData.range_min = 0.44;
  laserData.range_max = 3.0;
  laserData.ranges.resize(50);
  laserData.intensities.resize(50);

  for (auto& i : laserData.ranges) {
    i = 0.0;
  }

  PathPlanning planner;

  ros::Subscriber testSub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 50,
                                                               testCallback);

  int counter = 0;

  while (ros::ok()) {
    testPub.publish(laserData);
    planner.linearPathGenerator();

    if (counter == 3) {
      break;
    }
    ros::spinOnce();
    counter++;
  }

  EXPECT_NEAR(0.2, linX, 0.1);
  EXPECT_EQ(0.0, angZ);
}
