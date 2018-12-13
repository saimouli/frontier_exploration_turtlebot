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
 *@file           PathPlanning.h
 *@author         Saimouli Katragadda
 *@author         Saurav Kumar
 *@copyright      MIT License
 *@brief          PathPlanning class declaration
 *                Declares functions to publish spiral trajectories
 */

#ifndef INCLUDE_FRONTIER_EXPLORATION_TURTLEBOT_PATHPLANNING_H_
#define INCLUDE_FRONTIER_EXPLORATION_TURTLEBOT_PATHPLANNING_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <frontier_exploration_turtlebot/CollisionDetector.h>

/**
 * @brief PathPlanning Class
 * class to publish spiral trajectories and linear trajectories and kicks in the colloision avoiding algorithm when required
 *
 */
class PathPlanning {
 private:
  // Create CollisionDetector Object
  CollisionDetector collisiondetector;
  // variable to generate discrete spiral steps
  int count;
  // variable to declare maximum spiral step count
  int MaxCount;
  // variable to set the linear speed
  float linearSpeed;
  // variable to set the angular speed
  float angularSpeed;
  // declare a variable for velocities
  geometry_msgs::Twist msg;
  // publishes velocity
  ros::Publisher pubVel;
  // node handler
  ros::NodeHandle nh;
  // subscriber
  ros::Subscriber sub;

 public:
  /**
   * @brief constructor PathPlanning class
   * @param none
   * @return none
   * initializes values of count, MaxCount, linearSpeed,
   * angularSpeec and initializes subscribers and publishers
   */
  PathPlanning();

  /**
   * @brief destructor PathPlanning class
   * @param none
   * @return none
   * destroys PathPlanning class objects when
   * it goes out of scope.
   */
  ~PathPlanning();

  /**
   * @brief spiral generator function
   * @param none
   * @return none
   * generates spiral trajectories and publishes them
   */
  void spiralPathGenerator();

  /**
   * @brief linear generator function
   * @param none
   * @return none
   * generates linear trajectories and publishes them
   */
  void linearPathGenerator();
};

#endif  // INCLUDE_FRONTIER_EXPLORATION_TURTLEBOT_PATHPLANNING_H_
