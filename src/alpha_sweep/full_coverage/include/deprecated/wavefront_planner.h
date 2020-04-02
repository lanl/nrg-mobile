/*-------------------------------------------------------------------------------
 wavefront_planner.h

 Author: Alex von Sternberg
 Los Alamos National Laboratory

 Description: Determines full coverage path using wavefront algorithm
              given a grid of cells.
-------------------------------------------------------------------------------*/

#ifndef _WAVEFRONT_PLANNER_H_
#define _WAVEFRONT_PLANNER_H_

//ROS - Robot Operating System
//http://ros.org
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// Custom message types
#include "full_coverage/SquareCell.h"
#include "full_coverage/CellGrid.h"
#include "full_coverage/GetNavPlan.h"
#include "full_coverage/GenerateNavPlan.h"
#include "full_coverage/GetCoarseGrid.h"
#include "full_coverage/GetEmptyNeighbor.h"
#include "full_coverage/DebugGrid.h"

// Wavefront
#include "wavefront.h"

namespace full_coverage
{
  
/**
* Determines full coverage path
*/
class WavefrontPlanner {
private:
  // ROS infrastructure
  ros::NodeHandle n;                    /*!< ROS node handle */
  ros::ServiceServer planService;       /*!< ROS service server for starting a wavefront plan */
  ros::ServiceServer neighborService;   /*!< ROS service server for finding an unobstructed neighbor */
 
  // Contained objects
  Wavefront wave;                     /*!< Wave stores the wavefront data and contains all wavefront cells */
  
  // Initialization
  bool isInitialized;                 /*!< True if the class has been initialized */
  
  std::string poseFrame;              /*!< Frame of poses in plan */

  /**
   * @brief Callback function for the plan service.
   * @param req The service request message
   * @param res The service response message
   * @return True if plan was returned successfully
   */
  bool planCallback(full_coverage::GenerateNavPlan::Request &req, full_coverage::GenerateNavPlan::Response &res);
  bool getPlanCallback(full_coverage::GetNavPlan::Request &req, full_coverage::GetNavPlan::Response &res);

  /**
   * @brief Callback function for the get empty neighbor service.
   * @param req The service request message
   * @param res The service response message
   * @return True if executed successfully
   */
  bool getEmptyNeighborCallback(full_coverage::GetEmptyNeighbor::Request &req, full_coverage::GetEmptyNeighbor::Response &res);
  
  /**
   * @brief Attaches headers to poses.
   * @param poses Vector of poses to be converted to PoseStamped
   * @return converted poses
   */
  std::vector<geometry_msgs::PoseStamped> attachHeaders(std::vector<geometry_msgs::Pose> poses);
  
  /**
   * @brief Detaches headers from poses.
   * @param stampedPoses Vector of stampedPoses to be converted to Pose
   * @return converted poses
   */
  std::vector<geometry_msgs::Pose> detachHeaders(std::vector<geometry_msgs::PoseStamped> poses);
public:
  /**
   * @brief Constructor.
   */
  WavefrontPlanner();

  /**
   * @brief Destructor.
   */
  ~WavefrontPlanner();

}; // WavefrontPlanner

} // namespace full_coverage

#endif // _WAVEFRONT_H_
