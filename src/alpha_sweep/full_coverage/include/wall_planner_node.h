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
* Description: Determines full coverage path using a wall following 
*  algorithm given a grid of cells.
*********************************************************************/

#ifndef _WALL_PLANNER_NODE_H_
#define _WALL_PLANNER_NODE_H_

//ROS - Robot Operating System
//http://ros.org
#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "std_srvs/Empty.h"

// Custom message types
#include "full_coverage/SquareCell.h"
#include "full_coverage/CellGrid.h"
#include "full_coverage/GetNavPlan.h"
#include "full_coverage/GetCoarseGrid.h"
#include "full_coverage/BoolReq.h"
#include "full_coverage/PlanAction.h"
#include "full_coverage/Visit.h"
#include "full_coverage/DebugGrid.h"

// Wall planner
#include "wall_planner.h"

// tf
#include "tf/transform_listener.h"

// mutex
#include <mutex>

namespace full_coverage
{
  
/**
* Determines full coverage path
*/
class WallPlannerNode {
private:
  // ROS infrastructure
  ros::NodeHandle n;                    /*!< ROS node handle */
  tf::TransformListener listener;       /*!< TF transform listener */

  // ROS Communications
  actionlib::SimpleActionServer<full_coverage::PlanAction> planServer;  /*!< Plan server accepts plan commands and communicates result */
  full_coverage::PlanFeedback planFeedback;                             /*!< Feedback variable for plan server */
  full_coverage::PlanResult planResult;                                 /*!< Result variable for plan server */
  ros::ServiceServer getLastPlanService;                                /*!< ROS service server for getting the most recent plan */
  ros::ServiceServer monitorService;                                    /*!< ROS service server for monitoring which cells have been visited by the robot */
  ros::ServiceServer visitService;                                      /*!< ROS service server for visiting cells*/
  ros::ServiceServer clearService;                                      /*!< ROS service server for visiting cells*/
  ros::ServiceClient gridClient;                                        /*!< ROS service client for getting the coarse grid */
  ros::Subscriber    poseSub;                                           /*!< ROS subscriber for the current robot pose */
  geometry_msgs::Pose robotPose;                                        /*!< The current robot pose */
  ros::Publisher debugPub;

  // Contained objects
  WallPlanner *planner;               /*!< Wave stores the wavefront data and contains all wavefront cells */
  bool plannerReady;                  /*!< True if wall planner has been initialized and a valid plan exists */
  
  // Initialization and ROS parameters
  int occThreshold;                   /*!< Occupancy threshold for occupancy grid. Read in through ROS param server */
  int cellDim;                        /*!< Dimension of robot cell (in number of wavefront cells). Read in through ROS param server */
  int numBC;                          /*!< Number of border cells (cells along walls of objects that we will not plan for). Read in through ROS parameter server */
  bool isInitialized;                 /*!< True if the class has been initialized */
  bool isVisiting;

  // Plan variables
  std::string poseFrame;                  /*!< Frame of poses in plan */
  std::string lastFrame;                  /*!< Frame of poses in last plan */
  std::vector<geometry_msgs::Pose> plan;  /*!< Most recently generated plan */

  // Debug var
  bool debug;

  /**
   * @brief Callback function for retreiving the most recently generate plan.
   * @param req The service request message
   * @param res The service response message
   * @return True if plan was returned successfully
   */
  bool getLastPlanCallback(full_coverage::GetNavPlan::Request &req, full_coverage::GetNavPlan::Response &res);

  /**
   * @brief Callback function to start or stop monitoring service.
   * @param req The service request message
   * @param res The service response message
   * @return True if montior call was successful
   */
  bool monitorCallback(full_coverage::BoolReq::Request &req, full_coverage::BoolReq::Response &res);
  bool visitCallback(full_coverage::Visit::Request &req, full_coverage::Visit::Response &res);
  bool visit(std::vector<geometry_msgs::PoseStamped> attempts, std::vector<geometry_msgs::PoseStamped> visits);
  std::vector<geometry_msgs::PoseStamped> attemptQ;
  std::vector<geometry_msgs::PoseStamped> visitQ;
  std::mutex qMutex;
  bool clearCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  /**
   * @brief Callback function for storing the received robot pose.
   * @param msg The robot pose message
   */
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

  /**
   * @brief Callback function for simple action server plan execution.
   * @param goal The goal for the action server execution
   */
  void planExec(const full_coverage::PlanGoalConstPtr &goal);
  bool getCoarseGrid(full_coverage::CellGrid &cGrid);

  /**
   * @brief Attaches headers to poses.
   * @param poses Vector of poses to be converted to PoseStamped
   * @return converted poses
   */
  std::vector<geometry_msgs::PoseStamped> attachHeaders(std::vector<geometry_msgs::Pose> poses);

  /**
   * @brief Attaches a header to a pose (assumes the pose is in poseFrame)
   * @param pose Pose to be converted to stamped pose
   * @param i Value to put in pose.header.seq
   * @return converted pose
   */
  geometry_msgs::PoseStamped attachHeader(geometry_msgs::Pose pose, int i);

  /**
   * @brief Detaches headers from poses.
   * @param stampedPoses Vector of stampedPoses to be converted to Pose
   * @param pose Vector of poses that were converted
   * @return True if all conversions were successful
   */
  bool detachHeaders(std::vector<geometry_msgs::PoseStamped> stampedPoses, std::vector<geometry_msgs::Pose> &pose);

  /**
   * @brief Detaches header from pose.
   * @param stampedPose Stamped pose to be converted to a pose.
   * @param pose Converted pose
   * @return True if conversion was successfull
   */
  bool detachHeader(geometry_msgs::PoseStamped stampedPose, geometry_msgs::Pose &pose);

  /**
   * @brief Convenience method to convert a pose to the pose frame using TF
   * @param pose Pose to be converted
   * @return True if conversion was successful
   */
  bool toPoseFrame(geometry_msgs::PoseStamped &pose);
public:
  /**
   * @brief Constructor.
   */
  WallPlannerNode();

  /**
   * @brief Destructor.
   */
  ~WallPlannerNode();

  // Monitor checks current pose and marks what coverage cells have been visited by robot so we know what poses were missed.
  bool monitorCmd;
  void monitor();
}; // WallPlannerNode

} // namespace full_coverage

#endif // _WALL_PLANNER_NODE_H_
