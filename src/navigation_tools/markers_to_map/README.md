# Markers to Map

Projects a 3D map into 2D.

## Description

This package interacts with the 3D map from Rolling Map to produce a 2D occupancy map projection. It receives a marker array of all the occupied voxels in free space at different x, y, and z coordinates. From this marker array, it projects the occupancy into the x, y equivalent in a 2D occupancy map framework. It publishes the result as a nav_msgs::OccupancyGrid type.

## Instructions

TODO: Add detailed instructions for running standalone and alongside other nodes.