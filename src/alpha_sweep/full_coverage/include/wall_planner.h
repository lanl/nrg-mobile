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
* Author: Alex von Sternberg
*
* Description: Stores and calculates grid data for the wall follower 
*  planner.
*********************************************************************/
 
#ifndef _WALL_PLANNER_H_
#define _WALL_PLANNER_H_

// ROS header for ROS functionality
#include "ros/ros.h"

// WallPlanner and robot grid constructs
#include "coarse_grid.h"
#include "robot_grid.h"

// Map data construct
#include "full_coverage/CellGrid.h"
#include "full_coverage/DebugGrid.h"

// TF (ROS package for transforming poses)
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"

namespace full_coverage
{
  
/**
* WallPlanner construct
*/
class WallPlanner {
private:
  CoarseGrid coarseCells;          /*!< Vector of coarse cells */
  RobotGrid robotCells;            /*!< Vector of robot sized cells that contain coarse cells */
  int startIndex;                  /*!< Index of the robot cell that the plan should start at */
  bool isInitialized;              /*!< True if class was initialized successfully */
  tf::TransformListener listener;
  
    /**
   * @brief Adds the pose of the cell at the passed index to the plan accounting for orientation between the new cell and the previous cell in the plan.
   * @param lastI The index of the previous cell in the plan
   * @param i     The index of the cell to add to the plan
   * @param plan  The vector of poses that makes up the coverage plan
   * @return Number of new planned cells
   */
  int addPose(int i, MapDirection dir, std::vector<geometry_msgs::Pose>& plan, std::vector<bool> &planInfo, bool info = false);
  
    /**
   * @brief Sets the start index to the cell that startPose is contained in if it is a valid place to start.
   * @param startPose The pose that the robot at the start of the plan
   * @return True if the start index was set successfully
   */
  bool setStartIndex(geometry_msgs::Pose startPose);

  void patchCoarseGrid();

  void printPlan(std::vector<int> poses);
public:
  /**
   * @brief Constructor.
   */
  WallPlanner();

  /**
   * @brief Destructor.
   */
  ~WallPlanner();

  /**
   * @brief Initializes the wall planner.
   * @param map Map data
   * @param ot  Occupancy threshold for the map grid
   * @param rw  Numer of coarse cells that make up the width of the robot cell
   * @param rh  Numer of coarse cells that make up the height of the robot cell
   * @return True if initialization was successful
   */
  bool initialize(CellGrid map, int ot, int cd, int nbc);
  bool newMap(CellGrid map, int ot, int cd, int nbc, std::string poseFrame, std::string lastFrame);
  
  /**
   * @brief Propogates the cell values in the coarse and robot cells
   * @param startPose Pose of the robot at the start of the plan
   * @return True if the cell values were propogated successfully
   */
  bool fill(geometry_msgs::Pose startPose);
  
  /**
   * @brief Creates a full coverage plan based on the current cell values.
   * @param plan Stores the coverage plan
   * @param secAtt True if we are trying to plan attempted poses
   * @return True if a plan was possible
   */
  bool getPlan(std::vector<geometry_msgs::Pose> &plan, std::vector<bool> &planInfo, bool secAtt);
  bool replanFromPoses(std::vector<geometry_msgs::Pose> &poses);
  
  void visit(geometry_msgs::Pose robotPose);
  void setAttempted(std::vector<geometry_msgs::Pose> robotPose);

  bool initialized();

  double getCellWidth();

  full_coverage::DebugGrid getDebugMsg();

}; // WallPlanner

} // namespace full_coverage

#endif // _WALL_PLANNER_H_
