# Semantic Mapping Robot

## Overview

A ROS 2-based differential-drive robot capable of autonomously mapping, navigating, and identifying objects in its environment.

The robot uses **RPLIDAR and SLAM Toolbox** to generate a 2D occupancy map, while a camera and **YOLO-based object detection** pipeline identify objects and place labeled semantic markers on the map using LiDAR distance measurements and TF2 transformations.

Autonomous navigation is handled by **Nav2**, with motor control implemented through `ros2_control`, a custom hardware interface, and an Arduino controlling two NEMA 17 stepper motors.

## Features

* 2D LiDAR SLAM with SLAM Toolbox
* Autonomous navigation using Nav2
* YOLO camera-based object detection
* LiDAR-assisted object localization
* Semantic object markers displayed in RViz
* Custom `ros2_control` hardware interface
* Arduino-based stepper motor control
* ROS 2 communication between a Raspberry Pi and remote computer

## Technologies

* ROS 2 Jazzy
* SLAM Toolbox
* Nav2
* ros2_control
* TF2
* RViz
* YOLO / Ultralytics
* OpenCV
* Python / C++
* Raspberry Pi
* Arduino

## Status

**Completed — August 2026**

The final system integrates autonomous navigation, SLAM, computer vision, embedded motor control, and semantic mapping on a physical mobile robot.
