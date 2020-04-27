/*********************************************************************
*
*  © (or copyright) 2020. Triad National Security, LLC.
*  All rights reserved.
*  This program was produced under U.S. Government contract 
*  89233218CNA000001 for Los AlamosNational Laboratory (LANL), 
*  which is operated by Triad National Security, LLC for the U.S.
*  Department of Energy/National Nuclear Security Administration. 
*  All rights in the program are reserved by Triad National 
*  Security, LLC, and the U.S. Department of Energy/National Nuclear
*  Security Administration. The Government is granted for itself 
*  and others acting on its behalf a nonexclusive, paid-up, 
*  irrevocable worldwide license in this material to reproduce, 
*  prepare derivative works, distribute copies to the public, 
*  perform publicly and display publicly, and to permit others 
*  to do so.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Authors: Alex von Sternberg and Meredith Symmank
*
* Description: Converts an occupancy grid from the ROS navigation 
*  stack into a coarser grid that can be used for cellular 
*  decomposition and navigation purposes
*********************************************************************/

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
