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
#include <frontier_exploration_turtlebot/CollisionDetector.h>
#include <gtest/gtest.h>
#include <iostream>

/*
 *
 * @brief      Tests whether laserCallback method of the class CollisionDetector
 *
 * @param      CollisionDetectorTest     gtest framework
 * @param      CollisionDetectorTest     Name of the test
 */

TEST(CollisionDetectorTest, CollisionDetectorTest) {
  CollisionDetector collisionTest;
  std::cout << "TEST: " << collisionTest.checkObstacles() << std::endl;
  EXPECT_EQ(collisionTest.checkObstacles(), 0);
}

/**
 * @brief      Tests whether distanceCallback method of the class CollisionDetector
 *
 * @param      CollisionDetectorTest     gtest framework
 * @param      distanceCallbackTest      Name of the test
 */
TEST(CollisionDetectorTest, laserCallbackTest) {
  CollisionDetector collisionTest;
  ros::NodeHandle nh;
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

  ros::Publisher testPub = nh.advertise<sensor_msgs::LaserScan>("/scan", 50);

  ros::Subscriber testSub = nh.subscribe<sensor_msgs::LaserScan>(
      "/scan", 50, &CollisionDetector::laserCallback, &collisionTest);

  int collision = 1;
  int count = 0;

  while (ros::ok()) {
    testPub.publish(laserData);
    if (collisionTest.checkObstacles() == 0) {
      collision = 0;
      break;
    }
    ros::spinOnce();
    count++;
  }
  ROS_INFO_STREAM("count: " << count);
  ASSERT_FALSE(collision);
}
