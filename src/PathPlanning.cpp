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
 *@file PathPlanning.cpp
 *@author Saimouli Katragadda
 *@author Saurav Kumar
 *@copyright MIT License
 *@brief implements the PathPlanning class methods
 */
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "../include/frontier_exploration_turtlebot/CollisionDetector.h"
#include "../include/frontier_exploration_turtlebot/PathPlanning.h"

PathPlanning::PathPlanning() {
  ROS_INFO("Creating the Explorer behaviour...");
  // Set some parameters
  linearSpeed = 0.2;
  angularSpeed = 1;
  // Publish the velocity to cmd_vel
  pubVel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 50,
           &CollisionDetector::laserCallback, &collisiondetector);
  // Define the initial velocity message
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  count = 0;
  MaxCount = 15;
  // Stop the turtlebot
  pubVel.publish(msg);
}

PathPlanning::~PathPlanning() {
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  // Stop the turtlebot
  pubVel.publish(msg);
}

void PathPlanning::linearPathGenerator() {
  ros::Rate loop_rate(2);
  //  while (ros::ok()) {
    if (collisiondetector.checkObstacles() == 1) {
      msg.linear.x = 0;
      msg.angular.z = angularSpeed;
      // ROS_INFO_STREAM(
      // "collision : "<< collisiondetector.checkObstacles() <<" ,");
    }
    if (collisiondetector.checkObstacles() == 2) {
      msg.linear.x = linearSpeed;
      msg.angular.z = 0.0;
      //  ROS_INFO_STREAM(
      //  "collision : "<< collisiondetector.checkObstacles());
    }
    if (collisiondetector.checkObstacles() == 0) {
      msg.linear.x = linearSpeed;
      msg.angular.z = 0.0;
      //  ROS_INFO_STREAM(
      //  i << "," << count <<" ," << "linear : "
      //  << msg.linear.x << "angular : " << msg.angular.z);
    }

    pubVel.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
//}

void PathPlanning::spiralPathGenerator() {
  ros::Rate loop_rate(2);
  //  while (ros::ok()) {
    if (count == MaxCount) {
      count = 1;
    }

    for (int i = 1; i < count; i++) {
      if (i < count) {
        if (collisiondetector.checkObstacles() == 1) {
          msg.linear.x = 0;
          msg.angular.z = angularSpeed;
          //  ROS_INFO_STREAM(
          //  "collision : "<< collisiondetector.checkObstacles() <<" ,");
        }
        if (collisiondetector.checkObstacles() == 2) {
          msg.linear.x = linearSpeed;
          msg.angular.z = 0.0;
          //  ROS_INFO_STREAM(
          //  "collision : "<< collisiondetector.checkObstacles());
        }
        if (collisiondetector.checkObstacles() == 0) {
          msg.linear.x = linearSpeed;
          msg.angular.z = 0.0;
          //  ROS_INFO_STREAM(
          //  i << "," << count <<" ," << "linear : "
          //  << msg.linear.x << "angular : " << msg.angular.z);
        }

        pubVel.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();

        if (i == count - 1) {
          if (collisiondetector.checkObstacles() == 1) {
            msg.linear.x = 0;
            msg.angular.z = angularSpeed;
            //  ROS_INFO_STREAM(
            //  "collision : "<< collisiondetector.checkObstacles() <<" ,");
          }
          if (collisiondetector.checkObstacles() == 2) {
            msg.linear.x = linearSpeed;
            msg.angular.z = 0.0;
            //  ROS_INFO_STREAM(
            //  "collision : "<< collisiondetector.checkObstacles() <<" ,");
          }
          if (collisiondetector.checkObstacles() == 0) {
            msg.linear.x = 0.0;
            msg.angular.z = angularSpeed;
            //  ROS_INFO_STREAM(
            //  i << "," << count <<" ," << "linear : "<< msg.linear.x
            //  << "angular : " << msg.angular.z);
          }

          pubVel.publish(msg);
          ros::spinOnce();
          loop_rate.sleep();
        }
      }
    }
    count++;
  }
//}

