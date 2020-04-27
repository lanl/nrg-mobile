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

#include "wall_planner_node.h"

namespace full_coverage
{

WallPlannerNode::WallPlannerNode() :
  n("~"),
  planServer(n, "plan", boost::bind(&WallPlannerNode::planExec, this, _1), false),
  planner(0),
  plannerReady(false),
  occThreshold(0),
  cellDim(0),
  numBC(0),
  isInitialized(false),
  isVisiting(false),
  poseFrame(""),
  monitorCmd(false),
  debug(false)
{
  // Get parameters from ROS parameter server.
  if(!n.getParam("/Grid/occupancy_threshold", occThreshold))
  {
    ROS_ERROR_STREAM("Wall Planner Node: Could not get occupancy threshold from param server. Setting to default 10%");
    occThreshold = 10;
  }
  if(!n.getParam("/Grid/cell_dim", cellDim))
  {
    ROS_ERROR_STREAM("Wall Planner Node: Could not get robot width or height. Setting to default (3cellsx3cells");
    cellDim = 3;
  }
  if(!n.getParam("/Grid/num_border_cells",numBC))
  {
    ROS_ERROR_STREAM("Wall Planner Node: Could not get num border cells. Setting to default: 1.");
    numBC = 1;
  }
  n.param("/debug",debug,false);

  // Start plan action server
  planServer.start();
  planFeedback.percent_complete = 0.0;
  planResult.start_invalid = false;
  planResult.plan.poses.clear();
  planResult.plan.allow_skip.clear();

  // Create service client that will get map information from the coarse grid converter
  gridClient = n.serviceClient<full_coverage::GetCoarseGrid>("/coarse_grid_converter/get_coarse_grid");

  // Advertise services and subcribe to messages now that we have the data that we need to plan
  getLastPlanService = n.advertiseService("get_last_plan", &WallPlannerNode::getLastPlanCallback, this);
  monitorService = n.advertiseService("monitor", &WallPlannerNode::monitorCallback, this);
  visitService = n.advertiseService("visit", &WallPlannerNode::visitCallback, this);
  clearService = n.advertiseService("clear", &WallPlannerNode::clearCallback, this);

  // Subscribe to robot pose
  poseSub = n.subscribe("/robot_pose",1,&WallPlannerNode::poseCallback,this);

  // advertise debug message
  if(debug)
    debugPub = n.advertise<full_coverage::DebugGrid>("/debug_grid",1,true);

  // Node is initialized
  isInitialized = true;

  ROS_INFO("Wall Planner Node: Wall planner initialized");
} // WallPlannerNode

WallPlannerNode::~WallPlannerNode()
{
  if(planner)
    delete planner;
}

void WallPlannerNode::planExec(const full_coverage::PlanGoalConstPtr &goal)
{
  // Clear plan feedback and result variables
  planFeedback.percent_complete = 0.0;
  planResult.start_invalid = false;
  planResult.plan.poses.clear();
  planResult.plan.allow_skip.clear();
  planResult.text = "";

  // Do nothing if the planner was not initialized correctly
  if(!isInitialized)
  {
    planResult.text = "Wallfront has not been successfully initialized.";
    planServer.setAborted(planResult);
    ROS_ERROR("Wall Planner Node: Cannot get plan before successful initialization of node and planner.");
  }

  // If visit callback is still running, wait
  ros::Time startWait = ros::Time::now();
  while(isVisiting && ros::Time::now().toSec() - startWait.toSec() < 2.0 && ros::ok())
  {
    ros::Duration(0.1).sleep();
  }
  if(isVisiting)
    ROS_ERROR("Timed out waiting for visit callback. Some cells might not be visiited.");

  // Attempt to get the current coarse grid
  full_coverage::CellGrid cellGrid;  // stores service call request and response data
  if(!getCoarseGrid(cellGrid))
    return;

  // Reinitialize the planner
  if(planner)
  {
    ROS_INFO_STREAM("Wall Planner Node: Setting new map. Pose frame: " << poseFrame << " Last frame: " << lastFrame);
    if(!planner->newMap(cellGrid, occThreshold, cellDim, numBC, poseFrame, lastFrame))
    {
      planResult.text = "Failed to set new map in the planner";
      planServer.setAborted(planResult);
      ROS_ERROR("Wall Planner Node: Cannot set new map. Cannot create plan.");
      return;
    }
  }
  else
  {
    planner = new WallPlanner;

    // Initialize planner and send it the map data
    if(!planner->initialize(cellGrid, occThreshold, cellDim, numBC))
    {
      planResult.text = "Failed to initialize planner.";
      planServer.setAborted(planResult);
      ROS_ERROR("Wall Planner Node: Cannot initialize planner. Cannot create plan.");
      return;
    }
  }
  
  // Empty visit and attempted queues
  if(attemptQ.size() > 0 || visitQ.size() > 0)
  {
    ROS_INFO("Wall Planner Node: Visiting cells in queue");
    qMutex.lock();
    visit(attemptQ,visitQ);
    attemptQ.clear();
    visitQ.clear();
    qMutex.unlock();
  }

  // Transform the start pose to pose frame and set it in wall planner
  geometry_msgs::Pose transformedPose;
  if(!detachHeader(goal->start_pose,transformedPose))
  {
    planResult.start_invalid = true;
    planResult.text = "Start pose invalid.";
    planServer.setAborted(planResult);
    ROS_ERROR("Wall Planner Node: Cannot get navigation plan. Start pose could not be transformed to pose frame.");
    return;
  }

  // Fill wavefront grid so that a plan can be formed
  if(!planner->fill(transformedPose))
  {
    planResult.start_invalid = true;
    planResult.text = "Could not fill wave.";
    planServer.setAborted(planResult);
    ROS_ERROR("Wall Planner Node: Cannot get navigation plan. Failed to fill wave.");
    return;
  }

  /// for debuggung ///
  // publish the coarse grid
  if(debug)
  {
    full_coverage::DebugGrid msg = planner->getDebugMsg();
    msg.frame_id = poseFrame;
    debugPub.publish(msg);
  }

  // Make the plan
  std::vector<bool> skipInfo;
  if(!planner->getPlan(plan, skipInfo, goal->second_attempt))
  {
    planResult.text = "Could not get plan.";
    planServer.setAborted(planResult);
    ROS_ERROR("Wall Planner Node: Cannot get navigation plan. Get plan call failed.");
    return;
  }

  // Store plan in response variable and set server state to success
  plannerReady = true;
  planResult.plan.poses = attachHeaders(plan);
  planResult.plan.allow_skip.clear();
  for(int i = 0; i < skipInfo.size(); i++)
    planResult.plan.allow_skip.push_back(skipInfo[i]);
  planResult.plan.cell_width = planner->getCellWidth();
  planResult.text = "Plan generation successful.";
  planServer.setSucceeded(planResult);
}

bool WallPlannerNode::getCoarseGrid(full_coverage::CellGrid &cGrid)
{
  // Before calling the coarse grid service, wait until it is available
  full_coverage::GetCoarseGrid grid_srv;
  bool serviceExists = false;
  serviceExists = gridClient.waitForExistence(ros::Duration(5.0));

  // Abort if the service does not exist
  if(!serviceExists)
  {
    planResult.text = "Coarse grid service does not exist.";
    planServer.setAborted(planResult);
    ROS_ERROR("Wall Planner Node: Could not get coarse grid. Service does not exist.");
    return false;
  }

  // Use a service call to retrieve map data from get_coarse_grid service
  grid_srv.request.print_map = false;     // do not print map to the terminal
  bool gridReceived = false;
  ros::Time startTime = ros::Time::now();
  while(!gridReceived && startTime.toSec() - ros::Time::now().toSec() < 5.0)
  {
    if (gridClient.call(grid_srv)) // grid service call returns false if unsuccessful
    {
      gridReceived = true;
      lastFrame = poseFrame;
      poseFrame = grid_srv.response.cell_grid.frame_id;
      cGrid = grid_srv.response.cell_grid;
      ROS_INFO_STREAM("Pose frame is: " << poseFrame);
      return true;
    }
    else
    {
      ROS_INFO_THROTTLE(1.0, "Wall Planner Node: Waiting to receive coarse grid.");
    }
  }

  planResult.text = "Coarse grid service could not be called.";
  planServer.setAborted(planResult);
  ROS_ERROR("Wall Planner Node: Could not get coarse grid. Failed service call.");
  return false;
}

bool WallPlannerNode::clearCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  if(planner)
    delete planner;
  planner = 0;
  return true;
}

bool WallPlannerNode::getLastPlanCallback(full_coverage::GetNavPlan::Request &req, full_coverage::GetNavPlan::Response &res)
{
  if(plan.size()>0)
  {
    res.plan_poses = attachHeaders(plan);
    return true;
  }
  else
    return false;
}

void WallPlannerNode::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  if(poseFrame.compare("") != 0)
  {
    if(!detachHeader(*msg,robotPose))
    {
      ROS_ERROR_STREAM_THROTTLE(1.0, "Wall Planner Node: cannot convert robot pose: " << msg->header.frame_id << " to pose frame: " << poseFrame);
    }
  }
}

bool WallPlannerNode::monitorCallback(full_coverage::BoolReq::Request &req, full_coverage::BoolReq::Response &res)
{
  // we can't monitor if we didn't make a plan
  if(plannerReady || !req.data)
  {
    monitorCmd = req.data;
    return true;
  }
  else
  {
    ROS_ERROR("Wall Planner Node: Cannot monitor before planner is initialized.");
    return false;
  }
}

bool WallPlannerNode::visitCallback(full_coverage::Visit::Request &req, full_coverage::Visit::Response &res)
{
  isVisiting = true;
  if(!planServer.isActive() && plannerReady)
  {
    visit(req.attempted_poses,req.visit_poses);
  }  
  else
  {
    isVisiting = false;
    ROS_INFO("WallPlanner: Cannot visit at the moment because planner is not ready. Adding to visit queue.");
    qMutex.lock();
    if(req.attempted_poses.size() > 0)
      attemptQ.insert(attemptQ.end(), req.attempted_poses.begin(), req.attempted_poses.end());
    if(req.visit_poses.size() > 0)
      visitQ.insert(visitQ.end(), req.visit_poses.begin(), req.visit_poses.end());
    qMutex.unlock();
    return false;
  }
}

bool WallPlannerNode::visit(std::vector<geometry_msgs::PoseStamped> attempts, std::vector<geometry_msgs::PoseStamped> visits)
{
  std::vector<geometry_msgs::Pose> toVisit;
  if(detachHeaders(visits,toVisit))
  {
    for(int i = 0; i<toVisit.size(); i++)
    {
      planner->visit(toVisit[i]);
    }
    // If attempted poses is greater than zero then we will visit attempted poses that have already been attempted.
    // to not use this feature, always clear attempted poses vector
    if(attempts.size()>0)
    {
      std::vector<geometry_msgs::Pose> attempted;
      if(detachHeaders(attempts,attempted))
      {
        planner->setAttempted(attempted);
        ROS_INFO_STREAM("WallPlanner: Attempted " << attempted.size() << " cells.");
      }
      else
      {
        ROS_ERROR("WallPlanner: Skipping attempted poses in visit callback because could not detach headers");
      }
    }
    isVisiting = false;
    ROS_INFO_STREAM("WallPlanner: Visited " << toVisit.size() << " cells.");
    return true;
  }
  else
  {
    isVisiting = false;
    ROS_ERROR("WallPlanner: Skipping visited poses in visit callback because could not detach headers");
    return true;
  }
}


void WallPlannerNode::monitor()
{
  ROS_INFO("Wall planner: Monitor started.");
  ros::Rate loopRate(20.0);
  while(monitorCmd && plannerReady && ros::ok())
  {
    planner->visit(robotPose);
    loopRate.sleep();
  }

  if(!plannerReady)
  {
    ROS_ERROR("Stopping monitor because planner is not longer initialized");
    monitorCmd = false;
  }
}

std::vector<geometry_msgs::PoseStamped> WallPlannerNode::attachHeaders(std::vector<geometry_msgs::Pose> poses)
{
  std::vector<geometry_msgs::PoseStamped> stampedPoses;
  for(int i = 0; i<poses.size(); i++)
  {
    stampedPoses.push_back(attachHeader(poses[i],i));
  }
  return stampedPoses;
}

geometry_msgs::PoseStamped WallPlannerNode::attachHeader(geometry_msgs::Pose pose, int i)
{
  geometry_msgs::PoseStamped stampedPose;
  stampedPose.header.frame_id = poseFrame;
  stampedPose.header.seq = i;
  stampedPose.header.stamp = ros::Time::now();
  stampedPose.pose = pose;

  return stampedPose;
}

bool WallPlannerNode::detachHeaders(std::vector<geometry_msgs::PoseStamped> stampedPoses, std::vector<geometry_msgs::Pose> &poses)
{
  poses.clear();
  bool success = true;
  for(int i = 0; i<stampedPoses.size(); i++)
  {
    geometry_msgs::Pose pose;
    if(!detachHeader(stampedPoses[i],pose))
      success = false;
    poses.push_back(pose);
  }
  return success;
}

bool WallPlannerNode::detachHeader(geometry_msgs::PoseStamped stampedPose, geometry_msgs::Pose &pose)
{
  bool success = true;
  if(!toPoseFrame(stampedPose))
    success = false;

  pose = stampedPose.pose;
  return success;
}

bool WallPlannerNode::toPoseFrame(geometry_msgs::PoseStamped &pose)
{
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
  {
    return false;
  }
}

} // namespace full_coverage

// Main function initialized the node and creates the wall planner node instance
int main(int argc, char **argv)
{
  // Initialize node with name "wall_planner_node"
  ros::init(argc, argv, "wall_planner_node");

  // AsyncSpinner allows ROS functionality to execute when necessary
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Creates an instance of WallPlannerNode
  full_coverage::WallPlannerNode* wp = new full_coverage::WallPlannerNode();

  // Do not kill the node until ROS shutdown
  ros::Rate loopRate(1.0);
  while(ros::ok())
  {
    if(wp->monitorCmd)
    {
      wp->monitor();
    }
    loopRate.sleep();
  }

  return 0; // success
} // main
