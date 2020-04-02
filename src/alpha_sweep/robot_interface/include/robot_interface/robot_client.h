///////////////////////////////////////////////////////////////////////////////
//      Title     : robot_client.h
//      Author    : Alex von Sternberg, Los Alamos National Laboratory
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ROBOT_CLIENT_H
#define ROBOT_CLIENT_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <tf/transform_listener.h>

namespace robot_interface {

enum class GoalStatus {ACTIVE, SUCCESS, FAIL};

class RobotClient
{
  
public:
  /**
   * @brief Default constructor.
   * @param nh_ ROS NodeHandle
   * @param ros True if we are using ros navigation instead of ARNL
   */
  RobotClient(ros::NodeHandle* nh);
  
  /**
   * @brief Enable Pioneer drive motors. Blocks until complete or timeout elapses.
   * @return True if motors were enabled.
   */
  bool enableMotors();
  
  /**
   * @brief Disable Pioneer drive motors. Blocks until complete or timeout elapses.
   * @return True if motors were disabled.
   */
  bool disableMotors();

  bool areMotorsEnabled();

  /**
   * @brief Send a drive command to the Pioneer.
   * @param cmd The drive command.
   * TODO: Test to see if manual commands cancel nav goals on ROSArnl's end.
   */
  void sendDriveCmd(geometry_msgs::Twist cmd);
  void publishDrivePercent(geometry_msgs::Twist cmd);

  /**
   * @brief Send a ROS or ARNL navigation goal pose.
   * @param goal The goal pose.
   * @param block If true, this function blocks while the goal is active.
   * @return True if we reached the goal. Always returns true if block is false.
   */
  bool sendNavGoal(geometry_msgs::PoseStamped goal, bool block = false);

  /**
   * @brief Send a pose goal to be executed with straight line velocity commands.
   * @param goal The goal pose.
   * @param tol Tolerance on the velocity execution.
   * @param justTurn true if you just want to turn to the goal, and not drive to it
   * @return True if we reached the goal.
   */
  bool sendVelGoal(geometry_msgs::PoseStamped goal, double tol, bool justTurn = false, bool justDrive = false, bool jl = false);
  
  bool turnInPlace(double angle, double maxPercent);

  /**
   * @brief Find out if there is an active velocity goal.
   * @return True if a velocity goal is active.
   */
  bool isVelGoalActive();
  
  /**
   * @brief Cancel the active navigation goal, if any.
   */
  void cancelNavGoal();

  /**
   * @brief Cancel the active velocity goal
   */
  void cancelVelGoal();

  /**
   * @brief Stops the robot.
   */
  void stop();
  
  /**
   * @brief Get the status of the navigation goal.
   * @return The goal status
   */
  GoalStatus getNavGoalStatus();

  /**
   * @brief Get the current distance to the last goal
   * @param lin True if we want the linear distance, false for angular distance
   * @return The distance to the goal
   */
  double getNavGoalDistance(bool lin);
  double getDistance(geometry_msgs::PoseStamped target);


  /**
   * @brief Get the current robot pose.
   * @return The last received robot pose.
   */
  geometry_msgs::PoseStamped getRobotPose();
  
  /**
   * @brief Return the current battery status.
   * @return The battery status.
   */
  double getBatteryCharge() const;
  
  /**
   * @breif Shut down
   */
  void shutdown();

private:
  ros::NodeHandle* nh;
  tf::TransformListener listener;
  double commandRate;
  double commandTimeout;

  bool safeToDrive;
  bool safeToTurn;
  bool safeLaser;
  bool justLaser;
  bool stopCmd;


  double speedMax;
  double angularSpeedMax;
  
  geometry_msgs::PoseStamped robot_pose;
  geometry_msgs::PoseStamped last_target;
  bool setTarget(geometry_msgs::PoseStamped target);
  double battery_charge;

  ros::ServiceClient enable_motors_client;
  ros::ServiceClient disable_motors_client;
  ros::ServiceClient stop_client;

  
  ros::Publisher drive_cmd_pub;
  
  ros::Subscriber safe_drive_sub;
  ros::Subscriber safe_turn_sub;
  ros::Subscriber safe_laser_sub;
  ros::Subscriber robot_pose_sub;
  ros::Subscriber battery_sub;
  ros::Subscriber motor_state_sub;
  bool motorsEnabled;

  // Used for going to a pose via velocity commanding
  bool driveToGoal(double tol, double timeoutRatio, double maxPercent);
  bool turnToGoal(double tol, double timeoutRatio, double maxPercent);
  void getDesiredOffset(double &lin, double &rot);
  void stopVelocity();
  double getLinPercent(double maxPercent, bool provideDelta = false, double del = 0.0);
  double getRotPercent(double maxPercent, bool provideDelta = false, double del = 0.0);
  bool velActive;

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *navigation_server;
  
  /** Callback Functions **/
  void batteryChargeCallback(std_msgs::Float32 msg);

  void robotPoseCallback(geometry_msgs::PoseStamped msg);
  void safeDriveCallback(std_msgs::Bool msg);
  void safeTurnCallback(std_msgs::Bool msg);
  void safeLaserCallback(std_msgs::Bool msg);

  void motorStateCallback(std_msgs::Bool state);
  bool toPoseFrame(geometry_msgs::PoseStamped &pose);
  bool toBaseFrame(geometry_msgs::PoseStamped &pose);
};

} // namespace robot_interface

#endif // ROBOT_CLIENT_H
