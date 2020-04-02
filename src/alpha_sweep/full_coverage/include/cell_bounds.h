/*-------------------------------------------------------------------------------
 cell_bounds.h

 Author: Alex von Sternberg
 Los Alamos National Laboratory

 Description: Holds cell boundary data.
-------------------------------------------------------------------------------*/

#ifndef _CELL_BOUNDS_H_
#define _CELL_BOUNDS_H_

// ROS infrastructure
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "tf/transform_datatypes.h"

namespace full_coverage
{

class CellBounds
{
public:
  CellBounds();
  ~CellBounds();
  void setBounds(std::vector<geometry_msgs::Pose> allCorners);
  void getCorners(std::vector<geometry_msgs::Pose> &c);
  geometry_msgs::Pose getCenter();
  bool contains(geometry_msgs::Pose pose);
  double getWidth();

private:
  std::vector<tf::Point> corners;
  int shortest;
  int longest;
}; // CellBounds

} // namespace full_coverage

#endif // _CELL_BOUNDS_H_
