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
  ROS_INFO("Initializing Object Detection!");

  CollisionFlag = false;

  sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 50,
                                             &CollisionDetector::laserCallback,
                                             this);

  distancePub = nh.advertise<std_msgs::Float64>("/minDistance", 50);

  distanceSub = nh.subscribe<std_msgs::Float64>(
      "/minDistance", 50, &CollisionDetector::distanceCallback, this);
}

CollisionDetector::~CollisionDetector() {

}

void CollisionDetector::laserCallback(
    const sensor_msgs::LaserScan::ConstPtr& msg) {
  float threshold = 100;
  for (auto& i : msg->ranges) {
    if (i < threshold) {
      threshold = i;
    }
  }

  std_msgs::Float64 msgRange;
  msgRange.data = threshold;
  distancePub.publish(msgRange);
}

void CollisionDetector::distanceCallback(
    const std_msgs::Float64::ConstPtr& msg) {
  float minDist = 1.5;

  if ((msg->data) < minDist) {
    CollisionFlag = true;
    return;
  }
  CollisionFlag = false;
}

bool CollisionDetector::checkObstacles() {
  return CollisionFlag;
}
