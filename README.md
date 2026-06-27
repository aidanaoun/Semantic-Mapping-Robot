# Semantic Mapping Robot

> **Status:** Work in Progress
> **Expected Completion:** Early July 2026

## Overview

This project is a ROS 2-based differential drive robot designed to build a semantic map of its environment using LiDAR, camera perception, SLAM, and autonomous navigation.

The robot uses LiDAR data with `slam_toolbox` to create a 2D occupancy map, while camera-based perception is used to detect and label objects in the environment. Nav2 is used for autonomous navigation, and `ros2_control` is used for differential drive control.

## Technologies

* ROS 2
* SLAM Toolbox
* Nav2
* ros2_control
* Gazebo
* LiDAR
* Camera perception
* Python / C++
* Raspberry Pi

## Current Progress

* [x] ROS 2 workspace setup
* [x] Gazebo simulation
* [x] Differential drive control
* [x] Simulated LiDAR integration
* [x] Physical robot deployment
* [x] Real LiDAR integration
* [ ] Camera perception pipeline
* [ ] Semantic map labeling
* [ ] Final autonomous navigation demo

## Goal

The final goal is to deploy a physical mobile robot capable of autonomously mapping, navigating, and labeling objects in its environment using ROS 2.

This project is currently under active development, with the first complete version expected in **early July 2026**.
