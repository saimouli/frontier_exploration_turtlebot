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
 *@file         main.cpp
 *@author       Saimouli Katragadda
 *@author       Saurav Kumar
 *@copyright    MIT License
 *@brief        main function
 */

#include <ros/ros.h>
#include <frontier_exploration_turtlebot/PathPlanning.h>
#include <frontier_exploration_turtlebot/CollisionDetector.h>
#include <iostream>

/**
 * @brief      main function
 * @param      argc  The argc
 * @param      argv  The argv
 * @return     int of value zero
 */
int main(int argc, char* argv[]) {
  // Initialize the ros node
  ros::init(argc, argv, "frontierExplorer");
  // Variable to store user input
  int userChoice;
  std::cout << "Welcome to Turtlebot Explorer" << std::endl;
  std::cout << "Once you are satisfied with the map press (ctr+c)" << std::endl;
  std::cout << "To save the map rosrun map_server map_saver -f my_map"
            << std::endl;
  std::cout
      << "Would you like to take linear path (0) or spiral path finder (1)?"
      " (Enter 0 or 1): ";
  // read the userInput
  std::cin >> userChoice;
  // create the PathPlanning object
  PathPlanning pathPlanning;

  if (userChoice == 0) {
    // run the linear PathPlanning behaviour
    while (ros::ok()) {
      pathPlanning.linearPathGenerator();
    }
  }
  std::cin.get();
  if (userChoice == 1) {
    while (ros::ok()) {
      // run the spiral PathPlanning behaviour
      pathPlanning.spiralPathGenerator();
    }
  } else {
    // if the user inputs otherthan 0 or 1 initialize linear behaviour
    ROS_WARN_ONCE("Applying linearPathGenerator by default");
    while (ros::ok()) {
      pathPlanning.linearPathGenerator();
    }
  }

  return 0;
}
