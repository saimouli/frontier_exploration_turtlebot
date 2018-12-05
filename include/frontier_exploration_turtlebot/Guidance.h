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
 *@file Guidance.h
 *@author Saimouli Katragadda
 *@author Saurav Kumar
 *@copyright MIT License
 *@brief Guidance class declaration
 *Declares functions to publish the path velocities
 */

#ifndef GUIDANCE_H_
#define GUIDANCE_H_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

/**
 * @brief CollisonDetector Class
 * class to publish distance from the obstacle, find the presence of the obstacle
 * and coordinate other classes
 */
class Guidance {
 private:
  // subscribes the spiral trajectory
  ros::Subscriber pathSub;
  // node handler
  ros::NodeHandle nh;
  // declare message for velocities
  geometry_msgs::Twist msg;

 public:
  /**
   * @brief constructor Guidance class
   * @param none
   * @return none
   * Initializes subscribers and publishers
   */
  Guidance();

  /**
   * @brief destructor Guidance class
   * @param none
   * @return none
   * destroys Guidance class objects when
   * it goes out of scope.
   */
  ~Guidance();

  /**
   * @brief plan function
   * @param none
   * @return none
   * A subscribes to spiral path and publishes the final
   * velocities
   */
  void plan();
};



#endif /* GUIDANCE_H_ */
