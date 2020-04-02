/*-------------------------------------------------------------------------------
 markers_to_map.h

 Author: Alex von Sternberg
 Los Alamos National Laboratory

 Description: Takes in a marker array of occupied 3d space and outputs a 2d 
              costmap
-------------------------------------------------------------------------------*/

#ifndef _MARKERS_TO_MAP_H_
#define _MARKERS_TO_MAP_H_

//ROS - Robot Operating System
//http://ros.org
#include "ros/ros.h"

// tf
#include "tf/transform_listener.h"

// Message types
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"

// Mutex to stay thread safe
#include <mutex>

namespace markers_to_map
{
  
typedef std::map<int,visualization_msgs::Marker> MarkerMap;

class MarkersToMap {
private:
  // ROS infrastructure
  ros::NodeHandle n;                    /*!< ROS node handle */
  tf::TransformListener listener;       /*!< TF transform listener */

  // Parameters
  std::string mapFrame;
  std::string robotFrame;
  double resolution;
  double width;
  double height;
  double rate;

  // ROS subscribers
  ros::Subscriber markerSub;
  ros::Subscriber poseSub;

  // ROS publishers
  ros::Publisher mapPub;

  // State vars
  geometry_msgs::PoseStamped robotPose;
  MarkerMap markers;
  bool markerInit;

  // Mutex for current list of markers
  std::mutex markersMutex;

  // Callbacks
  void markerCb(const visualization_msgs::MarkerArray msg);
  void poseCb(const geometry_msgs::PoseStamped msg);

  // publish
  void publishMap();

  // Utility functions
  bool toFrame(geometry_msgs::PoseStamped &pose, std::string frame);
  bool isOccupied(int r, int c, const nav_msgs::OccupancyGrid &g);

public:
  /**
   * @brief Constructor.
   */
  MarkersToMap();

  /**
   * @brief Destructor.
   */
  ~MarkersToMap();

  void run();
}; // MarkersToMap

} // namespace markers_to_map

#endif // _MARKERS_TO_MAP_H_
