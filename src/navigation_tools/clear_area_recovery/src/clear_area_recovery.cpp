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
#include <clear_area_recovery/clear_area_recovery.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(clear_area_recovery::ClearAreaRecovery, nav_core::RecoveryBehavior)

namespace clear_area_recovery 
{

ClearAreaRecovery::ClearAreaRecovery():
  global_costmap_(NULL),
  local_costmap_(NULL), 
  tf_(NULL),
  initialized_(false) 
{
} 
    
ClearAreaRecovery::~ClearAreaRecovery()
{
  // do nothing
}
 
void ClearAreaRecovery::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap)
{
  ros::NodeHandle n("~" + name);
  local_costmap_ = local_costmap;
  tf_=tf;
  clearfootprintClient = n.serviceClient<rolling_map::Box>("/rolling_map_node/clear_box");
  height_ = 0.5;
  if(!n.getParam("height", height_))
  {
    ROS_ERROR("ClearAreaRecovery: Could not read height. Defaulting to .5");
  }
  padding_ = 1;
  if(!n.getParam("padding", padding_))
  {
    ROS_ERROR("ClearAreaRecovery: Could not read padding. Defaulting to 1.0");
  }

}

void ClearAreaRecovery::runBehavior()
{
  ROS_INFO("Executing clear area recovery behavior");

  // find the bounds of the robot footprint
  std::vector<geometry_msgs::Point> base = local_costmap_->getUnpaddedRobotFootprint();
  double xmin, xmax, ymin, ymax;
  if (base.size() > 0)
  {
    xmin = base[0].x;
    ymin = base[0].y;
    xmax = base[0].x;
    ymax = base[0].y;
   
    for (int i = 1; i < base.size(); i++)
    {
      if (base[i].x > xmax)
      {
        xmax = base[i].x;
      }
      if (base[i].x < xmin)
      {
        xmin = base[i].x;
      }
      if (base[i].y > ymax)
      {
        ymax = base[i].y;
      }
      if (base[i].y < ymin)
      {
        ymin = base[i].y;
      }
    }

  }

  // add padding
  double diff = xmax - xmin;
  diff = ((diff * padding_) - diff)/2.0;
  xmax = xmax + diff;
  xmin = xmin - diff;
  diff = ymax - ymin;
  diff = ((diff * padding_) - diff)/2.0;
  ymax = ymax + diff;
  ymin = ymin - diff;

  // call rolling map clear service
  rolling_map::Box foot_print;
  foot_print.request.p1.header.frame_id = local_costmap_->getBaseFrameID();
  foot_print.request.p2.header.frame_id = local_costmap_->getBaseFrameID();
  ROS_INFO_STREAM(local_costmap_->getBaseFrameID());
  foot_print.request.p1.point.x = xmin;
  foot_print.request.p2.point.x = xmax;
  foot_print.request.p1.point.y = ymin;
  foot_print.request.p2.point.y = ymax;
  foot_print.request.p1.point.z = 0;
  foot_print.request.p2.point.z = height_; 
  clearfootprintClient.call(foot_print);
}
}
