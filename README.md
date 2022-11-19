# ROS 2 beginner_tutorials

[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Overview

This repository contains beginner tutorials in C++ for a publisher and subscriber node in ROS

```
Talker node : src/publisher_member_function.cpp (Node for publishing)
Listener node : src/subscriber_member_function.cpp (Node for subscribing)
```

## Dependencies

- ROS 2 Foxy : [installation instructions](https://docs.ros.org/en/foxy/Installation.html)
- Ubuntu 20.04
- ament_cmake

## Build Instructions

```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
. install/setup.bash
cd src/
git clone https://github.com/adithyagaurav/beginner_tutorials
cd ..
colcon build --packages-select beginner_tutorials
```

## Running Instructions

Open a new terminal for running talker node

```
. install/setup.bash
ros2 run beginner_tutorials talker frequency:=10
```

Open a new terminal for running listener node

```
. install/setup.bash
ros2 run beginner_tutorials listener
```

Open a new terminal for running Service client

```
. install/setup.bash
ros2 run beginner_tutorials client CustomMessage
```

For running via launch file

```
. install/setup.bash
ros2 launch beginner_tutorials launch_service.yaml frequency:=60
```

