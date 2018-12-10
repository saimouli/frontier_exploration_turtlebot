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
 *@file CollisionDetector.cpp
 *@author Saimouli Katragadda
 *@author Saurav Kumar
 *@copyright MIT License
 *@brief implements the collisionDetector class methods
 */

#include "frontier_exploration_turtlebot/CollisionDetector.h"

CollisionDetector::CollisionDetector() {
  ROS_INFO("Initializing Collision Detection!");

  CollisionFlag = 0;

  //sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 50,
   //        &CollisionDetector::laserCallback, this);
}

CollisionDetector::~CollisionDetector() {
}

void CollisionDetector::laserCallback(
    const sensor_msgs::LaserScan::ConstPtr& msg) {

  std::vector<float> msgstore = msg->ranges;
  int countInd = 0;

  for(auto i : msgstore) {
    //ROS_INFO_STREAM("new auto: "<< i);
    int a = std::isinf(i);
    if (a==1) {
      // Replacing values in Inf values with max value 0.5
      msgstore[countInd] = 0.45;
    }
    countInd ++;
  }


// Replacing values in Inf values with max value 0.5
  // for (int i = 0; i <= 359; i++) {
  //   int a = std::isinf(msgstore[i]);
  //     if (a == 1) {
  //      msgstore[i] = 0.5;
  //      }
  // }

  float sumq1 = 0.0;
  float sumq2 =0.0;

  int counter = 0; // [0.2 0 0.3 0.5 0.3 0.5]

  for(auto i : msgstore) {
    if (i >= 0 && i <= 0.45) {
      //ROS_INFO_STREAM("ifloop ");
      if (counter >= 0 && counter < 45) {
        sumq1 = sumq1 + msgstore[counter]* 10;
        //ROS_INFO_STREAM("inside q1 " << sumq1 << " "<<counter);
      }
      if (counter >= 315 && i <= 359) {
        sumq1 = sumq1 + msgstore[counter] * 10;
         //ROS_INFO_STREAM("inside -45 45 q1 " << sumq1 << " "<<counter);
      }
      if (counter >= 135 && counter < 225) {
        sumq2 = sumq2 + msgstore[counter]* 10;
         //ROS_INFO_STREAM("inside q2 " << sumq2 << " "<<counter);
      }
  
    }
    counter++;
  }
  ROS_INFO_STREAM("Front obstacle area: " << sumq1);
  ROS_INFO_STREAM("Rear obstacle area: " << sumq2);


//   sumq1 = 0, sumq2 = 0;
//   for (int i = 0; i <= 359; i++) {
// //  ROS_INFO_STREAM(i);
//     if (msgstore[i] >= 0 && msgstore[i] <= 0.3) {
//       if (i >= 0 & i < 45) {
//         sumq1 = sumq1 + msgstore[i];
//       }
//       if (i >=315 & i <=359) {
//               sumq1 = sumq1 + msgstore[i];
//             }
//       if (i >= 135 & i< 225) {
//         sumq2 = sumq2 + msgstore[i];
//       }
//     }
//   }
//   ROS_INFO_STREAM("Front area : " << sumq1);
//   ROS_INFO_STREAM("Rear area : " << sumq2);

double minelement = std::min(sumq1, sumq2);
if (minelement == sumq1 && minelement < 30) {
  CollisionFlag = 1;
}
if (minelement == sumq2 && minelement < 30) {
  CollisionFlag = 2;
}
if (sumq1 == sumq2) {
  CollisionFlag = 0;
}
  }

int CollisionDetector::checkObstacles() {
  return CollisionFlag;
}

