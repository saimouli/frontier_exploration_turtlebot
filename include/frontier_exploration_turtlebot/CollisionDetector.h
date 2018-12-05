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
 *@file CollisionDetector.h
 *@author Saimouli Katragadda
 *@author Saurav Kumar
 *@copyright MIT License
 *@brief CollisionDetector class declaration
 *Declares functions to publish distance from the obstacle
 *and collision flag
 */

#ifndef COLLISIONDETECTOR_H_
#define COLLISIONDETECTOR_H_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"

/**
 * @brief CollisonDetector Class
 * class to publish distance from the obstacle, find the presence of the obstacle
 * and coordinate other classes
 */
class CollisionDetector {
 private:
  // variable to detect collision
  bool CollisionFlag;
  // node handler
  ros::NodeHandle nh;
  // subscribe laser scan topic
  ros::Subscriber sub;
  // subscribes range from laserscan
  ros::Subscriber distanceSub;
  // publishes distance from the robot to the obstacle
  ros::Publisher distancePub;

 public:
  /**
   * @brief constructor CollisionDetector class
   * @param none
   * @return none
   * initializes values of CollisionFlag and initializes
   * subscribers and publishers
   */
  CollisionDetector();

  /**
   * @brief destructor CollisionDetector class
   * @param none
   * @return none
   * destroys CollisionDetector class objects when
   * it goes out of scope.
   */
  ~CollisionDetector();

  /**
   * @brief laser callback function
   * @param msg of type sensor_msgs::LaserScan
   * @return none
   * A call back function to read laserscan data to update collision status
   */
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

  /**
   * @brief distance callback function
   * @param msg of type sensor_msgs::Float64
   * @return none
   * A call back function to read minimum distance
   * from the obstacle to the robot
   */
  void distanceCallback(const std_msgs::Float64::ConstPtr& msg);

  /**
   * @brief Check obstacles function
   * @param none
   * @return boolean
   * Funciton which checks if the obstacle is within the vicinity
   * of the robot and returns the CollisionFlag
   */
  bool checkObstacles();
};

#endif /* COLLISIONDETECTOR_H_ */
