/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <rotate_theta_recovery/rotate_theta_recovery.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(rotate_theta_recovery::RotateThetaRecovery, nav_core::RecoveryBehavior)

namespace rotate_theta_recovery 
{

RotateThetaRecovery::RotateThetaRecovery(): 
  global_costmap_(NULL), 
  local_costmap_(NULL), 
  initialized_(false), 
  world_model_(NULL) 
{
} 

void RotateThetaRecovery::initialize(
  std::string name, 
  tf2_ros::Buffer*, 
  costmap_2d::Costmap2DROS* global_costmap, 
  costmap_2d::Costmap2DROS* local_costmap)
{
  if(!initialized_)
  {
    name_ = name;
    //  = tf;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    //get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name_);
    ros::NodeHandle blp_nh("~/DWAPlannerROS");

    //we'll simulate every degree by default
    private_nh.param("sim_granularity", sim_granularity_, 0.017);
    private_nh.param("frequency", frequency_, 20.0);
    private_nh.param("rotate_theta", rotate_theta_, M_PI);
    
    acc_lim_th_ = 3.0;
    if(!blp_nh.getParam("acc_lim_theta", acc_lim_th_))
    {
      ROS_ERROR("RotateThetaRecovery: Could not read acc_lim_theta. Defaulting to 3.0");
    }
    
    max_rotational_vel_ = 0.5;
    if(!blp_nh.getParam("max_rot_vel", max_rotational_vel_))
    {
      ROS_ERROR("RotateThetaRecovery: Could not read max_rot_vel. Defaulting to 0.5");
    }
    
    min_rotational_vel_ = 0.15;
    if(!blp_nh.getParam("min_rot_vel", min_rotational_vel_))
    {
      ROS_ERROR("RotateThetaRecovery: Could not read min_rot_vel. Defaulting to 0.15");
    }

    tolerance_ = 0.10;
    if(!blp_nh.getParam("yaw_goal_tolerance", tolerance_))
    {
      ROS_ERROR("RotateThetaRecovery: Could not read yaw_goal_tolerance. Defaulting to 0.1");
    }

    if(rotate_theta_ < -M_PI || rotate_theta_ > M_PI) 
    {
      ROS_ERROR("Theta passed to RotateThetaRecovery must be between -pi and pi. normalizing angle.");
      rotate_theta_ = angles::normalize_angle(rotate_theta_);
    }

    world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

    initialized_ = true;
  }
  else
  {
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

RotateThetaRecovery::~RotateThetaRecovery()
{
  delete world_model_;
}

void RotateThetaRecovery::safeCb(std_msgs::Bool msg)
{
  safe_turn_ = msg.data;
}

void RotateThetaRecovery::runBehavior()
{
  if(!initialized_)
  {
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if(global_costmap_ == NULL || local_costmap_ == NULL)
  {
    ROS_ERROR("The costmaps passed to the RotateThetaRecovery object cannot be NULL. Doing nothing.");
    return;
  }
  ROS_WARN("Rotate recovery behavior started.");

  ros::Rate r(frequency_);
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  safe_turn_ = false;
  ros::Subscriber safe_sub = n.subscribe("/safe_drive_pub/safe_turn", 1, &RotateThetaRecovery::safeCb, this);

  // tf::Stamped<tf::Pose> global_pose;
  geometry_msgs::PoseStamped global_pose;
  local_costmap_->getRobotPose(global_pose);

  tf2::Quaternion robotQuat, startQuat;
  tf2::fromMsg(robotQuat, global_pose.pose.orientation);
  startQuat = robotQuat;
  double delta = fabs(robotQuat.angleShortestPath(startQuat));
  double sign = 1.0;
  if(rotate_theta_ < 0.0)
    sign = -1.0;
  ros::Time startTime = ros::Time::now();
  ros::Time lastMove = ros::Time::now();

  while(n.ok() && delta < fabs(rotate_theta_) && (ros::Time::now()-lastMove).toSec() < 30/frequency_ && (startTime - ros::Time::now()).toSec() < (2*M_PI)/max_rotational_vel_)
  {
    local_costmap_->getRobotPose(global_pose);
    // robotQuat = global_pose.getRotation();
    tf2::fromMsg(robotQuat, global_pose.pose.orientation);
    delta = fabs(robotQuat.angleShortestPath(startQuat));

    //compute the distance left to rotate
    double dist_left = fabs(rotate_theta_) - delta;

    //double x = global_pose.getOrigin().x(), y = global_pose.getOrigin().y();

    ////check if that velocity is legal by forward simulating
    //double sim_angle = 0.0;
    //while(sim_angle < dist_left){
    //  double theta = tf::getYaw(global_pose.getRotation()) + sign * sim_angle;

    //  //make sure that the point is legal, if it isn't... we'll abort
    //  double footprint_cost = world_model_->footprintCost(x, y, theta, local_costmap_->getRobotFootprint(), 0.0, 0.0);
    //  if(footprint_cost < 0.0){
    //    ROS_ERROR("Rotate recovery can't rotate in place because there is a potential collision. Cost: %.2f", footprint_cost);
    //    return;
    //  }

    //  sim_angle += sim_granularity_;
    //}

    if(safe_turn_)
    {
      //compute the velocity that will let us stop by the time we reach the goal
      double vel = sqrt(2 * acc_lim_th_ * dist_left);

      //make sure that this velocity falls within the specified limits
      if(vel < -max_rotational_vel_)
        vel = -max_rotational_vel_;
      else if (vel > max_rotational_vel_)
        vel = max_rotational_vel_;
      if(fabs(vel) > min_rotational_vel_)
      {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = sign*vel;

        vel_pub.publish(cmd_vel);
        lastMove = ros::Time::now();
      }
    }

    r.sleep();
  }
}
}
