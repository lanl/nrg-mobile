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
*********************************************************************/

#include <robot_interface/robot_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <algorithm>

namespace robot_interface
{

RobotClient::RobotClient(ros::NodeHandle* nh) :
  nh(nh),
  commandRate(10.0),
  commandTimeout(1.5),
  safeToDrive(false),
  safeToTurn(false),
  safeLaser(false),
  justLaser(false),
  stopCmd(false),
  velActive(false),
  motorsEnabled(false)
{  
  // Initialize nav server
  navigation_server = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("/move_base", true);

  // Service clients
  enable_motors_client = nh->serviceClient<std_srvs::Empty>("/robot/enable_motors");
  disable_motors_client = nh->serviceClient<std_srvs::Empty>("/robot/disable_motors");
  
  // Publishers
  drive_cmd_pub = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  // Subscribers
  battery_sub = nh->subscribe<std_msgs::Float32>("/robot/battery_state_of_charge", 10, &RobotClient::batteryChargeCallback, this);
  robot_pose_sub = nh->subscribe<geometry_msgs::PoseStamped>("/robot_pose", 1, &RobotClient::robotPoseCallback, this);
  safe_drive_sub = nh->subscribe<std_msgs::Bool>("/safe_drive_pub/safe_drive",1,&RobotClient::safeDriveCallback,this);
  safe_turn_sub = nh->subscribe<std_msgs::Bool>("/safe_drive_pub/safe_turn",1,&RobotClient::safeTurnCallback,this);
  safe_laser_sub = nh->subscribe<std_msgs::Bool>("/safe_drive_pub/safe_laser",1,&RobotClient::safeLaserCallback,this);
  motor_state_sub = nh->subscribe<std_msgs::Bool>("/robot/motors_state", 10, &RobotClient::motorStateCallback,this);

  // get ROS params
  if(!nh->getParam("max_lin_vel", speedMax))
  {
    ROS_ERROR("Robot Client: Max speed failed to load. Setting default value: 0.5");
    speedMax = 0.3;
  }
  if(!nh->getParam("max_rot_vel", angularSpeedMax))
  {
    ROS_ERROR("Robot Client: Max angular speed failed to load. Setting default value: 1.0");
    angularSpeedMax = 1.0;
  }
  if(!nh->getParam("/robot/command_rate", commandRate))
  {
    ROS_ERROR("Robot Client: Command rate failed to load. Setting default value: 10.0");
    commandRate = 10.0;
  }
  if(!nh->getParam("command_timeout", commandTimeout))
  {
    ROS_ERROR("Robot Client: Command timeout failed to load. Setting default value: 1.5");
    commandTimeout = 1.5;
  }

}

bool RobotClient::enableMotors()
{
  std_srvs::Empty srv;
  enable_motors_client.call(srv);

  ros::Time st = ros::Time::now();
  while(!motorsEnabled && ros::Time::now().toSec() - st.toSec() < 5.0)
  {
    ros::Duration(0.1).sleep();
    enable_motors_client.call(srv);
  }
  return motorsEnabled;
}

bool RobotClient::disableMotors()
{
  if(motorsEnabled)
  {
    std_srvs::Empty srv;
    disable_motors_client.call(srv);

    ros::Time st = ros::Time::now();
    while(motorsEnabled && ros::Time::now().toSec() - st.toSec() < 5.0)
    {
      ros::Duration(0.1).sleep();
    }
  }
  return !motorsEnabled;
}

bool RobotClient::areMotorsEnabled()
{
  return motorsEnabled;
}

void RobotClient::motorStateCallback(std_msgs::Bool state)
{
  motorsEnabled = state.data;
}



void RobotClient::sendDriveCmd(geometry_msgs::Twist cmd)
{
  bool safeCmd = true;
  if(!safeToTurn && (fabs(cmd.angular.z) > 0.001 && fabs(cmd.linear.x < 0.005)))
    safeCmd = false;
  if(((!safeToDrive && !justLaser) || (!safeLaser && justLaser)) && fabs(cmd.linear.x) > 0.001)
    safeCmd = false;
  if(safeCmd)
    drive_cmd_pub.publish(cmd);
  else
    stopVelocity();
}

void RobotClient::publishDrivePercent(geometry_msgs::Twist cmd)
{
  cmd.linear.x = cmd.linear.x/100.0 * speedMax;
  cmd.linear.y = cmd.linear.y/100.0 * speedMax;
  cmd.linear.z = cmd.linear.z/100.0 * speedMax;
  cmd.angular.x = cmd.angular.x/100.0 * angularSpeedMax;
  cmd.angular.y = cmd.angular.y/100.0 * angularSpeedMax;
  cmd.angular.z = cmd.angular.z/100.0 * angularSpeedMax;
  sendDriveCmd(cmd);
}

bool RobotClient::turnInPlace(double angle, double maxPercent)
{
  velActive = true;

  ROS_INFO("Robot Base Client: Turning in place.");
  tf::Quaternion robotQuat, startQuat;
  tf::quaternionMsgToTF(robot_pose.pose.orientation, robotQuat);
  startQuat = robotQuat;

  // Send stop to allow drive commanding
  stop();
  stopCmd = false;

  // turn left *angle*
  ros::Rate loopRate(commandRate);
  std::vector<double> distHist;
  double tol = 0.9;
  double delta = robotQuat.angleShortestPath(startQuat);
  distHist.push_back(fabs(delta) + 1.0);
  distHist.push_back(fabs(delta));
  auto minmax = std::minmax_element(distHist.begin(),distHist.end());
  double range = fabs(*(minmax.first)-*(minmax.second));
  while(fabs(delta) < angle*tol && ros::ok() && range > 0.01 && !stopCmd)
  {
    // publish the drive command
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = getRotPercent(maxPercent, true, angle-fabs(delta));
    publishDrivePercent(cmd);

    distHist.push_back(fabs(delta));
    if(distHist.size() > round(commandTimeout*commandRate))
      distHist.erase(distHist.begin());
    minmax = std::minmax_element(distHist.begin(),distHist.end());
    range = fabs(*(minmax.first)-*(minmax.second));
    //std::cout << "Delta: " << delta << " Min: " << *(minmax.first) << " Max: " << *(minmax.second) << " Range: " << range << std::endl;

    tf::quaternionMsgToTF(robot_pose.pose.orientation, robotQuat);
    delta = robotQuat.angleShortestPath(startQuat);

    loopRate.sleep();
  }

  // turn back to start
  distHist.push_back(fabs(delta) + 1.0);
  distHist.erase(distHist.begin());
  while(fabs(delta) > 0.07 && ros::ok() && range > 0.01 && !stopCmd)
  {
    // publish the drive command
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = getRotPercent(maxPercent, true, -delta);
    publishDrivePercent(cmd);

    distHist.push_back(fabs(delta));
    if(distHist.size() > round(commandTimeout*commandRate))
      distHist.erase(distHist.begin());
    minmax = std::minmax_element(distHist.begin(),distHist.end());
    range = fabs(*(minmax.first)-*(minmax.second));
    //std::cout << "Delta: " << delta << " Min: " << *(minmax.first) << " Max: " << *(minmax.second) << " Range: " << range << std::endl;

    tf::quaternionMsgToTF(robot_pose.pose.orientation, robotQuat);
    delta = robotQuat.angleShortestPath(startQuat);

    loopRate.sleep();
  }

  // turn rigth *angle*
  distHist.push_back(fabs(delta) + 1.0);
  distHist.erase(distHist.begin());
  while(fabs(delta) < angle*tol && ros::ok() && range > 0.01 && !stopCmd)
  {
    // publish the drive command
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = getRotPercent(maxPercent, true, -angle-fabs(delta));
    publishDrivePercent(cmd);

    distHist.push_back(fabs(delta));
    if(distHist.size() > round(commandTimeout*commandRate))
      distHist.erase(distHist.begin());
    minmax = std::minmax_element(distHist.begin(),distHist.end());
    range = fabs(*(minmax.first)-*(minmax.second));
    //std::cout << "Delta: " << delta << " Min: " << *(minmax.first) << " Max: " << *(minmax.second) << " Range: " << range << std::endl;

    tf::quaternionMsgToTF(robot_pose.pose.orientation, robotQuat);
    delta = robotQuat.angleShortestPath(startQuat);

    loopRate.sleep();
  }

  stopVelocity();
  velActive = false;
  return true;
}

bool RobotClient::sendVelGoal(geometry_msgs::PoseStamped goal, double tol, bool justTurn, bool justDrive, bool jl)
{
  justLaser = jl;
  velActive = true;

  ROS_INFO("Robot Base Client: Executing velocity goal.");
  if(!setTarget(goal))
  {
    ROS_ERROR("RobotClient: sendVelGoal(): failed to set target.");
    justLaser = false;
    velActive = false;
    return false;
  }

  // Send stop to allow drive commanding
  stop();
  stopCmd = false;

  if(!justDrive)
  {
    double ttol = 0.03;
    if(justTurn)
      ttol = tol;
    // Change orientation to point to goal
    if(!turnToGoal(ttol,1.5,100.0))
    {
      ROS_ERROR("Failed to turn to goal");
      velActive = false;
      justLaser = false;
      return false;
    }
    else
    {
      ROS_INFO("Turned to goal.");
    }
  }

  if(!justTurn)
  {
    // Move forward to goal
    if(!driveToGoal(tol, 1.5, 25.0))
    {
      ROS_ERROR("Failed to drive to goal");
      velActive = false;
      justLaser = false;
      return false;
    }
    else
    {
      ROS_INFO("Drove to goal.");
    }
  }

  justLaser = false;
  velActive = false;
  if((!justTurn && fabs(getNavGoalDistance(true)) < tol) || (justTurn && fabs(getNavGoalDistance(false) < tol)))
    return true;
  else
    return false;
}

bool RobotClient::isVelGoalActive()
{
  return velActive;
}

bool RobotClient::turnToGoal(double tol, double timeoutRatio, double maxPercent)
{
  ros::Rate loopRate(commandRate);
  std::vector<double> distHist;
  distHist.push_back(fabs(getNavGoalDistance(false)) + 1.0);
  distHist.push_back(fabs(getNavGoalDistance(false)));
  while(fabs(getNavGoalDistance(false)) > tol && ros::ok() && fabs(fabs(getNavGoalDistance(false))-distHist[0]) > tol*0.1 && !stopCmd)
  {
    // publish the drive command
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = getRotPercent(maxPercent);
    publishDrivePercent(cmd);

    distHist.push_back(fabs(getNavGoalDistance(false)));
    if(distHist.size() > round(commandTimeout*commandRate))
      distHist.erase(distHist.begin());

    loopRate.sleep();
  }

  stopVelocity();

  if(getNavGoalDistance(false) <= tol)
    return true;
  else
    return false;
}

bool RobotClient::driveToGoal(double tol, double timeoutRatio, double maxPercent)
{
  ros::Rate loopRate(commandRate);
  std::vector<double> distHist;
  distHist.push_back(fabs(getNavGoalDistance(true)) + 1.0);
  distHist.push_back(fabs(getNavGoalDistance(true)));
  while(fabs(getNavGoalDistance(true)) > tol && ros::ok() && fabs(fabs(getNavGoalDistance(true))-distHist[0]) > tol*0.1 && !stopCmd)
  {
    // publish the drive command
    geometry_msgs::Twist cmd;
    cmd.linear.x = getLinPercent(maxPercent);
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = getRotPercent(maxPercent);
    publishDrivePercent(cmd);

    distHist.push_back(fabs(getNavGoalDistance(true)));
    if(distHist.size() > round(commandTimeout*commandRate))
      distHist.erase(distHist.begin());

    loopRate.sleep();
  }

  stopVelocity();

  if(getNavGoalDistance(true) <= tol)
    return true;
  else
    return false;
}

double RobotClient::getLinPercent(double maxPercent, bool provideDelta, double del)
{
  double delta = 0.0;
  if(provideDelta)
  {
    delta = del;
  }
  else
  {
    delta = getNavGoalDistance(true);
    double rot = getNavGoalDistance(false);
    getDesiredOffset(delta,rot);
  }
  if(delta > 2.0)
    return maxPercent;
  else if(delta > 1.0)
  {
    if(20.0 < maxPercent)
      return 20.0;
    else
      return maxPercent;
  }
  else if(delta > 0.50)
  {
    double temp = 21.0*delta;
    if(temp < 11.0)
      return 11.0;
    else
      return temp;
  }
  else if(delta > 0.01)
    return 11.0;
  else if(delta > -0.01)
    return 0.0;
  else
    return -11.0;
}

double RobotClient::getRotPercent(double maxPercent, bool provideDelta, double del)
{
  double delta = 0.0;
  if(provideDelta)
  {
    delta = del;
  }
  else
  {
    delta = getNavGoalDistance(false);
  }
  if(delta > M_PI/8.0)
    return maxPercent;
  else if(delta > M_PI/16.0)
    return 25.0;
  else if(delta > M_PI/64.0)
    return (64.0*5.0/M_PI)*delta + 5.0;
  else if(delta > M_PI/128.0)
    return (1280.0/M_PI)*delta - 6.0;
  else if(delta > M_PI/256.0)
    return 4.0;
  else if(delta > -M_PI/256.0)
    return 0.0;
  else if(delta > -M_PI/128.0)
    return -4.0;
  else if(delta > -M_PI/64.0)
    return (1280.0/M_PI)*delta + 6.0;
  else if(delta > -M_PI/16.0)
    return (64.0*5.0/M_PI)*delta - 5.0;
  else if(delta > -M_PI/8.0)
    return -25.0;
  else
    return -maxPercent;
}

void RobotClient::getDesiredOffset(double &lin, double &rot)
{
  if(fabs(rot) > M_PI/2.0)
  {
    lin = 0.0;
  }
}

void RobotClient::stopVelocity()
{
  geometry_msgs::Twist cmd;
  cmd.linear.x = 0.0;
  cmd.linear.y = 0.0;
  cmd.linear.z = 0.0;
  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;
  cmd.angular.z = 0.0;
  drive_cmd_pub.publish(cmd);
}

bool RobotClient::sendNavGoal(geometry_msgs::PoseStamped goal, bool block)
{
  stopCmd = false;

  ROS_INFO("Robot Base Client: Sending nav goal.");
  move_base_msgs::MoveBaseGoal action_goal;
  action_goal.target_pose = goal;
  if(!setTarget(goal))
  {
    ROS_ERROR("RobotClient: sendNavGoal(): failed to set target");
    return false;
  }
  
  if (block) {
    actionlib::SimpleClientGoalState result = navigation_server->sendGoalAndWait(action_goal);
    return result.state_ == actionlib::SimpleClientGoalState::SUCCEEDED;
  } else {
    navigation_server->sendGoal(action_goal);
    return true;
  }
}

double RobotClient::getNavGoalDistance(bool lin)
{
  geometry_msgs::PoseStamped target = last_target;
  if(!toBaseFrame(target))
  {
    ROS_ERROR("Robot Base Client: Cannot get nav goal distance. Cannot convert target pose to base_link frame.");
    return 0.0;
  }
  if(lin)
  {
    return pow(pow(target.pose.position.x,2) +
               pow(target.pose.position.y,2) , 0.5);
  }
  else
  {
    return atan2(target.pose.position.y,target.pose.position.x);
  }
}

double RobotClient::getDistance(geometry_msgs::PoseStamped target)
{
  if(!toBaseFrame(target))
  {
    ROS_ERROR("Robot Base Client: Cannot get goal distance. Cannot convert target pose to base_link frame.");
    return 0.0;
  }
  return pow(pow(target.pose.position.x,2) +
             pow(target.pose.position.y,2) , 0.5);
}

void RobotClient::cancelVelGoal()
{
  stopCmd = true;
}

void RobotClient::cancelNavGoal()
{
  if(navigation_server->getState().state_ == actionlib::SimpleClientGoalState::ACTIVE || navigation_server->getState().state_ == actionlib::SimpleClientGoalState::PENDING)
    navigation_server->cancelGoal();
}

void RobotClient::stop()
{
  stopCmd = true;
  cancelNavGoal();
  stopVelocity();
}

GoalStatus RobotClient::getNavGoalStatus()
{
  const actionlib::SimpleClientGoalState result = navigation_server->getState();
  
  // Goal is in progress
  if (!result.isDone()) {
    return GoalStatus::ACTIVE;
  }
  
  // Success
  if (result == actionlib::SimpleClientGoalState::SUCCEEDED) {
    return GoalStatus::SUCCESS;
  }
  
  // Failure
  return GoalStatus::FAIL;
}

geometry_msgs::PoseStamped RobotClient::getRobotPose()
{
  return robot_pose;
}

double RobotClient::getBatteryCharge() const
{
  return battery_charge;
}

void RobotClient::batteryChargeCallback(std_msgs::Float32 msg)
{
  battery_charge = msg.data;
}

void RobotClient::robotPoseCallback(geometry_msgs::PoseStamped msg)
{
  robot_pose = msg;
}

void RobotClient::safeDriveCallback(std_msgs::Bool msg)
{
  safeToDrive = msg.data;
}

void RobotClient::safeTurnCallback(std_msgs::Bool msg)
{
  safeToTurn = msg.data;
}

void RobotClient::safeLaserCallback(std_msgs::Bool msg)
{
  safeLaser = msg.data;
}

bool RobotClient::setTarget(geometry_msgs::PoseStamped target)
{
  if(!toPoseFrame(target))
    return false;

  std::cout << "RobotInterface: Set target to: x: " << target.pose.position.x << " y: " << target.pose.position.y << std::endl;
  last_target = target;
  return true;
}

bool RobotClient::toPoseFrame(geometry_msgs::PoseStamped &pose)
{
  std::string poseFrame = robot_pose.header.frame_id;
  if(poseFrame.compare("")!=0)
  {
    if(pose.header.frame_id.compare(poseFrame) != 0)
    {
      tf::Stamped<tf::Pose> in, out;
      pose.header.stamp = ros::Time(0);
      tf::poseStampedMsgToTF(pose, in);
      try
      {
        if(listener.waitForTransform(poseFrame, pose.header.frame_id, ros::Time(0), ros::Duration(1.0)))
        {
          listener.transformPose(poseFrame, in, out);
          tf::poseStampedTFToMsg(out, pose);
          return true;
        }
        else
        {
          return false;
        }
      }
      catch(...)
      {
        return false;
      }
    }
    else
    {
      // Do nothing, the pose is already in map frame
      return true;
    }
  }
  else
    return false;
}

bool RobotClient::toBaseFrame(geometry_msgs::PoseStamped &pose)
{
  std::string baseFrame = "base_link";
  if(pose.header.frame_id.compare(baseFrame) != 0)
  {
    tf::Stamped<tf::Pose> in, out;
    pose.header.stamp = ros::Time(0);
    tf::poseStampedMsgToTF(pose, in);
    try
    {
      if(listener.waitForTransform(baseFrame, pose.header.frame_id, ros::Time(0), ros::Duration(1.0)))
      {
        listener.transformPose(baseFrame, in, out);
        tf::poseStampedTFToMsg(out, pose);
        return true;
      }
      else
      {
        return false;
      }
    }
    catch(...)
    {
      return false;
    }
  }
  else
  {
    // Do nothing, the pose is already in map frame
    return true;
  }
}

} // namespace robot_interface
