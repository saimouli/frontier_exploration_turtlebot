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
/**
 *@file           CollisionDetectorTest.cpp
 *@author         Saimouli Katragadda
 *@author         Saurav Kumar
 *@copyright      MIT License
 *@brief          Tests the collisionDetector class methods
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <frontier_exploration_turtlebot/CollisionDetector.h>
#include <gtest/gtest.h>
#include <iostream>

/**
 * @brief      Tests constructor of the class CollisionDetector
 * @param      CollisionDetectorTest     gtest framework
 * @param      CollisionDetectorTest     Name of the test
 * @return     none
 */

TEST(CollisionDetectorTest, CollisionDetectorTest) {
  CollisionDetector collisionTest;
  std::cout << "TEST: " << collisionTest.checkObstacles() << std::endl;
  EXPECT_EQ(collisionTest.checkObstacles(), 0);
}

/**
 * @brief      Tests laserCallback method of the class CollisionDetector
 * @param      CollisionDetectorTest     gtest framework
 * @param      distanceCallbackTest      Name of the test
 * @return     none
 */
TEST(CollisionDetectorTest, laserCallbackTest) {
  // initialize the collisionTest object
  CollisionDetector collisionTest;
  ros::NodeHandle nh;
  //  create a laserData test msg to send the custom laserData
  sensor_msgs::LaserScan laserData;
  laserData.angle_min = -0.52;
  laserData.angle_max = 0.52;
  laserData.angle_increment = 0.0016;
  laserData.time_increment = 0.0;
  laserData.range_min = 0.44;
  laserData.range_max = 3.0;
  laserData.ranges.resize(50);
  laserData.intensities.resize(50);

  //  setting all values in the range msg to zeros
  for (auto& i : laserData.ranges) {
    i = 0.0;
  }
  // creating Publisher testpub which publishes to scan topic
  ros::Publisher testPub = nh.advertise<sensor_msgs::LaserScan>("/scan", 50);
  //  creating Subscriber testsub  subscribing to scan topic
  // and calling lasercallback function of CollisionDetector class
  ros::Subscriber testSub = nh.subscribe<sensor_msgs::LaserScan>(
      "/scan", 50, &CollisionDetector::laserCallback, &collisionTest);

  // setting collision to flase
  int collision = 1;
  int count = 0;

  while (ros::ok()) {
    // publish the test custom laserData
    testPub.publish(laserData);
    // check to see if the has set the collisionFlag
    if (collisionTest.checkObstacles() == 0) {
      // if the collisionFLag is triggered set the collision to true
      collision = 0;
      break;
    }
    ros::spinOnce();
    count++;
  }
  // chek to see if the retured collision is false or not
  ASSERT_FALSE(collision);
}
