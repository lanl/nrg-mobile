# Rolling Map

Takes in LIDAR sensor data and provides a 3D map of an area in a rolling window as a robot navigates. 

## Description

This package contains two nodes and an optional CUDA file for running on a GPU. The rolling map package is designed to process input pointcloud data from the Velodyne LIDAR sensor and output a map that is viewable through [RViz](http://wiki.ros.org/rviz). 

It is configurable through a .yaml file.

## Dependencies

Requires PCL via ROS
CUDA recommended (see instructions below)

## Instructions

### Running with CUDA

It is necessary to have a GPU in order for the CUDA implementation to have the proper affect. 

To run with CUDA, the following is necessary. Access the CMakeList.txt file.

Change the line

```
set(CUDA_ENABLE FALSE)
```

to 

```
set(CUDA_ENABLE TRUE)
```

TODO: Add detailed instructions for running this package.