/*-------------------------------------------------------------------------------
 coarse_grid_converter.h
 Version: 1.0
 Commit Date: 07/06/2016

 Authors: Meredith Pitsch
 The University of Texas at Austin
 Department of Mechanical Engineering
 Nuclear and Applied Robotics Group

 Description: Converts an occupancy grid from the ROS navigation stack
 into a coarser grid that can be used for cellular decomposition
 and navigation purposes

 Command Line Arguments: None
-------------------------------------------------------------------------------*/

#ifndef _COARSE_GRID_CONVERTER_H_
#define _COARSE_GRID_CONVERTER_H_

//ROS - Robot Operating System
//http://ros.org
#include <ros/ros.h>
#include <string>
#include <sstream>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <tf/transform_broadcaster.h>
#include <algorithm>

// Custom message types
#include "full_coverage/SquareCell.h"
#include "full_coverage/CellGrid.h"
#include "full_coverage/GetCoarseGrid.h"

// open cv for image rotation
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace full_coverage {
   
/**
* Performs the grid map conversion
*/
class CoarseGridConverter {
private:
  bool mapAvailable;
  bool costmapAvailable;
  nav_msgs::OccupancyGrid publishedMap;
  nav_msgs::OccupancyGrid publishedCostmap;
  
  /**
   * Standard ROS node handle.
   */
  ros::NodeHandle n;
  ros::AsyncSpinner spinner;

  tf::TransformBroadcaster broadcaster;
  tf::StampedTransform transform;
  tf::StampedTransform previousTransform;
  bool broadcastTransform;
  bool broadcastPreviousTransform;
  bool useCostmap;

  /**
   * Standard ROS subscriber for fetching the occupancy map from the
   * navigation stack
   */
  ros::Subscriber mapSub;
  ros::Subscriber costmapSub;
  ros::Subscriber costmapUpdatesSub;

   /**
   * Standard ROS service server for advertising the coarse grid from 
   * the conversion
   */
  ros::ServiceServer getGridServer; 

  /**
   * Desired output occupancy grid from the conversion
   */
  full_coverage::CellGrid navGrid;

  /**
   * @brief Subscriber Callback to convert the nav stack grid
   * @param grid The navigation stack occupancy grid to be converted
   */
  void mapCallback(nav_msgs::OccupancyGrid grid);
  void costmapCallback(nav_msgs::OccupancyGrid grid);
  void costmapUpdatesCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& gridUpdate);
  void parseMap(nav_msgs::OccupancyGrid globalGrid, nav_msgs::OccupancyGrid localGrid);
  void findRotation(const nav_msgs::OccupancyGrid &grid, double &mapRotation);
  void rotateMap(nav_msgs::OccupancyGrid &grid, const double mapRotation);
  void overlayLocal(nav_msgs::OccupancyGrid &globalGrid, const nav_msgs::OccupancyGrid &localGrid);
  int transformCount;

  /**
   * @brief Service Callback to return the coarse output grid
   * @param req A boolean indicating whether the service should print an 
   *   example map to the terminal
   * @param res The returned grid map
   */
  bool getGridCallback(full_coverage::GetCoarseGrid::Request &req, full_coverage::GetCoarseGrid::Response &res);

  /**
   * @brief Creates a cell with the information from a set of pixels
   *   from the nav stack occupancy grid
   * @param row    The new cell's row index
   * @param col    The new cell's column index
   * @param cw     The new cell's width in pixels
   * @param ch     The new cell's height in pixels
   * @param res    The new resolution of the grid (m/pix)
   * @param op     The average occupancy probability of the cell from the comprising pixels
   * @param origin The origin of the occupancy grid in the map
   */
  full_coverage::SquareCell fillCell(int row, int col, int cw, int ch, double res, int op, geometry_msgs::Pose origin);
  
  /**
   * @brief Optional function to print a rough diagram of the coarse grid 
   *   to the terminal
   */  
  void printNewGrid(void);
  ros::Publisher rotPub;
    
public:
  /**
   * @brief Constructor to the class that converts a pixelated ROS 
   *   occupancy grid into a coarse grid map for navigation. 
   */
  CoarseGridConverter();

  /**
   * @brief Desctructor for the class 
   */
  ~CoarseGridConverter();

  void updateTransform();

}; // CoarseGridConverter

} // namespace full_coverage

#endif // _COARSE_GRID_CONVERTER_H_
