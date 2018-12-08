# Frontier Exploration With Turtlebot
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Build Status](https://travis-ci.org/saimouli/frontier_exploration_turtlebot.svg?branch=master)](https://travis-ci.org/saimouli/frontier_exploration_turtlebot)
[![Coverage Status](https://coveralls.io/repos/github/saimouli/frontier_exploration_turtlebot/badge.svg?branch=master)](https://coveralls.io/github/saimouli/frontier_exploration_turtlebot?branch=master)

# Authors 
- Saurav Kumar
- Saimouli Katragadda

This package is developed by following an agile software development technique called pair programming (where a pair of programmers take turns writing and inspecting code), a product backlog and iteration logs is also documented and links are below. Additionally, Test Driven Development (TDD) where a development cycle with repetitive testing, writing code, and code refactoring process will befollowed.

# Overview
A map of the environment plays a crucial role in our robot’s navigation, guidance and path
planning module. With the changing interiors in a room and each time letting the user map
manually is tedious. Having the robot to explore the entire room autonomously to build a map is very
essential for a product to be commercially successful. The objective of this project is to build a basic
turtlebot based frontier exploration package capable of autonomously navigating and building a map in a confined
room until the user is satisfied with the output using ROS and C++.

# License
```
MIT License

Copyright (c) 2018 Saimouli Katragadda, Saurav Kumar

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

# Product Backlog and Sprint Schedule
The product backlog can be accessed [here](https://docs.google.com/spreadsheets/d/1aUEmwQJSr9hcHAbKPmq_1oXtsOK8NiAJHHwCcbalw80/edit#gid=0) to access the sprint schedule 

The Sprint Planning and review can be accessed [here](https://docs.google.com/document/d/19h8M4_cV-hC2vH-sakYy-WoBOPhtShtMgyjMKj_OG6o/edit?usp=sharing)

Saimouli Katragadda and Saurav Kumar worked together on this implementation and alternate commits were made. The commits are made by the driver while the other person acted as navigator.

# Dependencies
This project uses the following packages: 
- ROS Kinetic
- Turtlebot ROS packages
- Hector_slam packages
- Ubuntu 16.04
 
# Algorithm Overview 
There are two path planning generators called   ```Linear Path Genenrator``` and ```Spiral Path Generator``` to generate the motion of the turtlebot in the simulated obstacle free environment. The ```Linear Path Generator``` generates a linear path for the turtlebot in the collision free area. Whereas, the later generates discrete spiral path. ```CollisionDetecter``` overrides ```Linear Path Generator``` and ```Spiral Path Generator``` whenever an object is in the vicinity of the turtlebot.


The ```CollisionDetecter``` algorithm divides the vicinity of the turtelbot into two areas : ```Front``` & ```Rear```. It then compares the obstacles in these two vicinities and make a decision either to move forward or to rotate. <TODDO: eloborate the algorithm>

The ```Linear Path Genenrator``` or ```Spiral Path Generator``` kicks in back once both of the vicinities are obstacle free.

# Build Instructions
To run follow the following commands 
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone --recursive https://github.com/saimouli/frontier_exploration_turtlebot.git
cd ..
catkin_make
```

# Sourcing
source the following in bashrc by typing the following 
```
gedit ~/.bashrc
source ~/catkin_ws/devel/setup.bash
source ~/.bashrc
```
Below stubs will be updated in the future 
# Run Tests
```
cd ~/catkin_ws/
```

# Running Demo 
```
cd ~/catkin_w
```
# Doxygen Documentation 

# Coverage 

# Recording ROSBAG

## Running ROSBAG

# About Authors 
## Saurav
Saurav is pursuing Masters in Robotics Engineering.He has five years of experience in research and development field in automobile sector at Daimler.He is passionate about autonomous driving system such as self steering,cross track error minimization, and traffic detection. Refer Linkedin profile [here](https://www.linkedin.com/in/saurav-kumar-2a532242/).

## Saimouli 
Saimouli is pursuing Robotic Engineering at UMD. He is passionate about flying cars, drones, and deep learning. In future, he would like design and create autonomous flights. Outside of academics, he is involved in Autonomous Micro Aerial Vehicle Team and loves to watch movies and make short films in his free time. 
