/*-------------------------------------------------------------------------------
 cover_map.h
 Version: 2.0
 Commit Date: 01/12/2017

 Authors:
 Alex von Sternberg
 Los Alamos National Lab

 Meredith Pitsch
 The University of Texas at Austin Nuclear Robotics Group

 Description: Executes a navigation plan to fully cover a  map based on a
 grid structure. Specifially intended for the Pioneer-lx system.
-------------------------------------------------------------------------------*/

#ifndef _COVER_MAP_H_
#define _COVER_MAP_H_

//ROS - Robot Operating System
//http://ros.org
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>

// std vector, iterator, string, future (for async)
#include <string>
#include <vector>
#include <iterator>
#include <future>

// Pioneer interface
#include "robot_interface/robot_client.h"

// Custom message types
#include "full_coverage/ExecuteAction.h"
#include "full_coverage/BoolReq.h"

// enums
#include "coverage_enums.h"

namespace full_coverage {
typedef actionlib::SimpleActionServer<full_coverage::ExecuteAction> ExecutionServer;
typedef std::vector<std::pair<geometry_msgs::PoseStamped,bool>> fcPlan;

/**
* Cover Map Class.
* Performs execution of a full coverage plan.
*/
class CoverMap {
private:
  // ROS Communications
  ros::NodeHandle n;                              //!< ROS node handle for registering ROS communications
  ros::AsyncSpinner spinner;
  tf::TransformListener *listener;
  ros::ServiceClient monitorClient;               //!< ROS Service client tells the coverage planner to start/stop monitoring coverage
  ExecutionServer executeServer;                  //!< ROS action server for executing/stopping/resuming cover map plan
  full_coverage::ExecuteFeedback executeFeedback; //!< action server feedback variable to be send back to client
  full_coverage::ExecuteResult executeResult;     //!< action server result variable sent to client at action termination
  ros::Publisher targetPub;                       //!< ROS publisher of current target pose
  ros::Subscriber mapSub;
  ros::Subscriber mapUpdatesSub;
  nav_msgs::OccupancyGrid map;
  bool gotMap;
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void mapUpdatesCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg);

  // Robot Interface
  robot_interface::RobotClient robot;               //!< Interface to robot_interface for commanding/telemetry
  // geometry_msgs::PoseStamped dockLocation;    //!< Dock location is always 0,0,0 in dock frame
  // geometry_msgs::PoseStamped dockApproach;    //!< Dock approach is always 1,0,0 in dock frame

  // State vars
  double cellWidth;
  std::vector<std::pair<geometry_msgs::PoseStamped,bool>> plan;                    //!< Coverage plan (member var for pause/resume capabilities)
  std::vector<std::pair<geometry_msgs::PoseStamped,bool>>::iterator planPos;       //!< Current position in plan
  CommandMode cmdMode;                                             //!< Current mode of command (Velocity cmds or navigation cmds)
  bool lastZag;  // for keeping track of if we are skipping for zig zags (in which case we don't want to jump straight into line skipping)
  bool lastLine; // for keeping track of if we are skipping for lines (in which case we don't want to jump straight into zig zag skipping)
  bool lastSkip; // set this to true if the last pose was skipped (so we can keep track of the first pose in a group of skips)
  std::vector<std::pair<geometry_msgs::PoseStamped,bool>>::iterator skipPos; // the first pose in a line of skips
  std::vector<std::pair<geometry_msgs::PoseStamped,bool>>::iterator skipResetPos; // the first pose in a line of skips

  /**
   * @brief Action server execution callback to execute a coverage plan.
   * @param goal Goal of the execution (Type of command, plan poses).
   */
  void executeCallback(const full_coverage::ExecuteGoalConstPtr& goal);
  void startCoverage();
  void stopRobot();
  void monitor(bool toMonitor);
  void executeTurn();
  // void executeDock();
  // void executeUndock();
  bool checkPlanVars();
  bool initializePlanVars(const full_coverage::ExecuteGoalConstPtr& goal);
  void markPoses(bool visited, fcPlan::iterator lastSuccess, bool previousSuccess = false);


  /**
   * @brief inLineX checks if the two parameter poses are in the same position in the x dimension
   * @param pose1 One of the poses to compare
   * @param pose2 The other pose to compare
   * @return True if the 2 poses are in line
   */
  bool inLineX(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2);

  /**
   * @brief inLineY checks if the two parameter poses are in the same position in the y dimension
   * @param pose1 One of the poses to compare
   * @param pose2 The other pose to compare
   * @return True if the 2 poses are in line
   */
  bool inLineY(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2);
  bool toSkip(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2, geometry_msgs::PoseStamped pose3, geometry_msgs::PoseStamped zagRef);
  bool nextTo(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2);
  bool checkPose(geometry_msgs::PoseStamped pose);

  /**
   * @brief Accesses the ROS navigation stack or custom velocity commanding to attempt to move to
   *   a given goal point.
   * @param pose The pose to which the robot must attempt to travel
   * @return True if the robot successfully got to the commanded pose
   */
  bool goToPoint(geometry_msgs::PoseStamped pose);

  bool asyncNavGoal(geometry_msgs::PoseStamped pose, double tol);
  bool asyncVelGoal(geometry_msgs::PoseStamped pose, double tol, bool justTurn = false, bool justDrive = false, bool justLaser = false);
  // bool asyncDockReverse(double desired, double tol);

  fcPlan::iterator getProgressPos(fcPlan::iterator skipP, fcPlan::iterator planP);
  bool transformPose(geometry_msgs::PoseStamped &pose, std::string to_frame);

public:
  /**
   * @brief Constructor
   */
  CoverMap();

  /**
   * @brief Destructor
   */
  ~CoverMap();

}; // CoverMap

} // namespace full_coverage

#endif // _COVER_MAP_H_
