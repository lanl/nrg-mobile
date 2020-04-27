# Velodyne Orient

Changes the velodyne frame based on the point cloud input so that the ground plane can be easily clipped.

## Description

This package contains one ROS node, velodyne_orient.cpp, for transforming the frame of pointclouds received from the velodyne sensor such that they can be more easily processed. The main purpose of the transformation is to remove the ground plane from consideration as an obstacle. It runs as a background process and activates whenever a new pointcloud message is published.

A configuration file is included for tuning parameters. It is accessible through ROS's [dynamic reconfigure](http://wiki.ros.org/dynamic_reconfigure) framework.

## Dependencies

Requires PCL via ROS

## Instructions

To run standalone, a launch file has been provided. Use it with 

```bash
roslaunch velodyne_orient velodyne_orient.launch
```
TODO: Add step by step instructions for running alongside other nodes