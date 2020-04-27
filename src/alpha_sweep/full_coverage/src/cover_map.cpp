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
* Description: Executes a navigation plan to fully cover a  map
*  based on a grid structure. 
*********************************************************************/

#include "cover_map.h"

namespace full_coverage
{

CoverMap::CoverMap() :
  n("~"),
  spinner(3),
  gotMap(false),
  robot(&n),
  executeServer(n, "execute", boost::bind(&CoverMap::executeCallback, this, _1), false),
  cellWidth(0.0),
  cmdMode(CMD_VEL),
  lastZag(false),
  lastLine(false),
  lastSkip(false)
{
  // Load ROS config variables
  std::string mt = "/move_base/global_costmap/costmap";
  if(!n.getParam("map_topic", mt))
  {
    ROS_ERROR("CoverMap: Could not read map topic. Defaulting to /move_base/global_costmap/costmap");
  }
  bool cm = true;
  if(!n.getParam("is_costmap", cm))
  {
    ROS_ERROR("CoverMap: Could not read is_costmap parameter. Defaulting to true");
  }

  // Setup ROS communications
  monitorClient = n.serviceClient<full_coverage::BoolReq>("/wall_planner_node/monitor");
  targetPub = n.advertise<geometry_msgs::PoseStamped>("target",1,true);
  listener = new tf::TransformListener();
  mapSub = n.subscribe<nav_msgs::OccupancyGrid>(mt,1,&CoverMap::mapCallback,this);
  if(cm)
    mapUpdatesSub = n.subscribe<map_msgs::OccupancyGridUpdate>(mt + "_updates",1,&CoverMap::mapUpdatesCallback,this);


  // Start the async spinner
  spinner.start();

  // Start the execution action server
  executeServer.start();
}  // CoverMap

CoverMap::~CoverMap() {
  delete listener;
}  // ~CoverMap

void CoverMap::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  map = *msg;
  gotMap = true;
}

void CoverMap::mapUpdatesCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg)
{
  // If we don't have a full map yet, do nothing
  if(!gotMap)
  {
    return;
  }

  // Reject updates which have any out-of-bounds data.
  if(msg->x < 0 ||
     msg->y < 0 ||
     map.info.width < msg->x + msg->width ||
     map.info.height < msg->y + msg->height )
  {
    return;
  }

  // Copy the incoming data into map's data.
  for( size_t y = 0; y < msg->height; y++ )
  {
    memcpy( &map.data[ (msg->y + y) * map.info.width + msg->x ],
            &msg->data[ y * msg->width ],
            msg->width );
  }
}

void CoverMap::executeCallback(const full_coverage::ExecuteGoalConstPtr& goal)
{
  // Reset result variables
  executeResult.type = goal->type;
  executeResult.text = "";

  // Check which type of goal we are handling
  if(goal->type == full_coverage::ExecuteGoal::STOP)
  {
    ROS_INFO("Cover map: New stop command.");

    // Stop robot and then exit callback
    stopRobot();
    return;
  }
  else if(goal->type == full_coverage::ExecuteGoal::TURN)
  {
    ROS_INFO("Cover map: New turn command.");

    // Execute a turn in place and then exit callback
    executeTurn();
    return;
  }
  // else if(goal->type == full_coverage::ExecuteGoal::DOCK)
  // {
  //   ROS_INFO("Cover map: New dock command.");

  //   // Execute dock procedure then exit callback
  //   executeDock();
  //   return;
  // }
  // else if(goal->type == full_coverage::ExecuteGoal::UNDOCK)
  // {
  //   ROS_INFO("Cover map: New undock command.");

  //   // Execute undock procedure then exit callback
  //   executeUndock();
  //   return;
  // }
  else if(goal->type == full_coverage::ExecuteGoal::RESUME)
  {
    ROS_INFO("Cover map: New resume command");

    // Check that plan variables are still valid. If they are, continue into main execution loop
    if(!checkPlanVars())
      return;
  }
  else
  {
    ROS_INFO("Cover map: New start command");

    // If it is a start goal, initialize plan vars, and continue into the main execution loop
    if(!initializePlanVars(goal))
      return;
  }

  // Main coverage loop
  startCoverage();
}

void CoverMap::startCoverage()
{
  // Initialize robot
  if(!robot.enableMotors())
  {
    robot.stop();
    executeResult.text = "Execution failed because we could not enable motors.";
    executeServer.setAborted(executeResult);
    return;
  }

  // Start coverage monitoring so all cells the robot visits are counted
  monitor(true);

  // First pose should not be straight line velocity execution since
  // we do not know if obstacles are in between us and the pose
  cmdMode = CMD_POSE;

  fcPlan::iterator lastSuccess = planPos;
  bool previousSuccess = false;

  // Iterate until we are at the end of the plan
  while(planPos != plan.end())
  {
    // Check for cancel command or preempt
    if(executeServer.isPreemptRequested() || !ros::ok())
    {
      robot.stop();
      executeResult.text = "Plan execution was canceled or paused.";
      executeServer.setPreempted(executeResult);
      return;
    }

    // Check the pose in the costmap. If cost is too high, there was probably new data since the plan was formed and we should quit and make a new plan.
    if(!checkPose((*planPos).first))
    {
      // Turn off coverage monitor after execution
      monitor(false);

      // mark the pose as attempted
      executeFeedback.attempted_poses.clear();
      executeFeedback.visited_poses.clear();
      executeFeedback.attempted_poses.push_back((*planPos).first);
      executeServer.publishFeedback(executeFeedback); 

      // Send success
      executeResult.text = "Plan execution was stopped becuase plan pose had high cost.";
      executeServer.setSucceeded(executeResult);
      return;
    }

    // If we are not on our first attempt, skip poses that are in line with previous and next pose
    bool skip = false; // set to true if we want to skip this pose
    if((*planPos).second && (planPos != plan.begin() && planPos != (plan.end()-1) && planPos != plan.end()))
    {
      geometry_msgs::PoseStamped poseTemp;
      if(planPos == (plan.begin() +1))
        poseTemp = (*planPos).first;
      else
        poseTemp = (*(planPos-2)).first;
      skip = toSkip((*(planPos-1)).first,(*planPos).first,(*(planPos+1)).first,poseTemp);
    }

    if(!skip)
    {
      // Make sure last vars are false
      lastLine = false;
      lastZag = false;

      // Publish current target pose
      ROS_INFO_NAMED("CoverMap", "CoverMap: Attempting to reach cell at %5.2f, %5.2f", (*planPos).first.pose.position.x, (*planPos).first.pose.position.y);
      targetPub.publish((*planPos).first);

      // Move robot to the pose
      if(goToPoint((*planPos).first))
      {
        // If we made it successfully, switch back to velocity commanding and increment to the next pose
        ROS_INFO_NAMED("CoverMap", "CoverMap: Moved to cell");
        cmdMode = CMD_VEL;
        markPoses(true, lastSuccess, previousSuccess);
        previousSuccess = true;
        lastSuccess = planPos;
        planPos++;
      }
      else
      {
        if(CMD_VEL == cmdMode)
        {
          // If we failed with velocity commanding, switch to nav commanding and try again. Also, set plan pos to the last pose that we skipped.
          cmdMode = CMD_POSE;
          if(lastSkip) // only set plan pos to skipped pose if we actually skipped a pose
          {
            planPos = getProgressPos(skipPos, planPos);
            // avoid infinite retry loop
            if(std::distance(plan.begin(),planPos) == std::distance(plan.begin(),skipResetPos))
              planPos++;
            skipResetPos = planPos;
          }
        }
        else
        {
          // If we failed with both types of commanding, move on to the next pose
          ROS_ERROR_NAMED("CoverMap", "CoverMap: Failed to reach cell at %5.2f, %5.2f", (*planPos).first.pose.position.x, (*planPos).first.pose.position.y);
          markPoses(false, lastSuccess);
          planPos++;
        }
        previousSuccess = false;
      }

      // set last skip to false
      lastSkip = false;
    }
    else
    {
      // set last skip to true and record skip pos if necessary
      if(!lastSkip)
        skipPos = planPos;
      lastSkip = true;

      // Skip to the next pose
      planPos++;
    }
  }

  // Turn off coverage monitor after execution
  monitor(false);

  // Set succeeded
  executeResult.text = "Plan execution finished successfully";
  executeServer.setSucceeded(executeResult);
} // executeCallback

void CoverMap::markPoses(bool visited, fcPlan::iterator lastSuccess, bool previousSuccess)
{
  executeFeedback.attempted_poses.clear();
  executeFeedback.visited_poses.clear();
  if((lastSuccess+1)!= planPos && lastSuccess != planPos && lastSuccess != plan.end() && (lastSuccess+1) != plan.end() && (previousSuccess || !visited))
  {
    fcPlan tempVec(lastSuccess+1, planPos+1);
    for(int i = 0; i<tempVec.size(); i++)
    {
      if(visited)
        executeFeedback.visited_poses.push_back(tempVec[i].first);
      else
        executeFeedback.attempted_poses.push_back(tempVec[i].first);
    }
  }
  else
  {
    if(visited)
      executeFeedback.visited_poses.push_back((*planPos).first);
    else
      executeFeedback.attempted_poses.push_back((*planPos).first);
  }
  executeServer.publishFeedback(executeFeedback); 
}

void CoverMap::monitor(bool toMonitor)
{
  full_coverage::BoolReq srv;
  srv.request.data = toMonitor;
  monitorClient.call(srv);
}

void CoverMap::stopRobot()
{
  // Stop monitor
  monitor(false);

  // If it is a stop goal, stop the robot and return success
  robot.stop();
  executeResult.text = "Robot stopped";
  executeServer.setSucceeded(executeResult);
}

void CoverMap::executeTurn()
{
  // Initialize robot
  if(!robot.enableMotors())
  {
    robot.stop();
    executeResult.text = "Execution failed because we could not enable motors.";
    executeServer.setAborted(executeResult);
    return;
  }
  
  // turn in place in new thread
  std::future<bool> success = std::async(std::launch::async, &robot_interface::RobotClient::turnInPlace, &robot, M_PI/3,40.0);
  // sleep so vel active flag is set
  ros::Duration(0.2).sleep();
  ros::Rate attemptRate(10.0);
  // Wait for goal to complete
  while(robot.isVelGoalActive())
  {
    if(executeServer.isPreemptRequested() || !ros::ok())
    {
      robot.stop();
      executeResult.text = "Plan execution was canceled or paused.";
      executeServer.setPreempted(executeResult);
      return;
    }
    attemptRate.sleep();
  }

  // Sleep to let gmapping udpate
  ros::Duration(0.5).sleep();

  // return success/failure
  bool wasSucc = success.get();
  if(wasSucc)
  {
    executeResult.text = "Robot turned in place";
    executeServer.setSucceeded(executeResult);
    return;
  }
  else
  {
    executeResult.text = "Robot failed to turn in place.";
    executeServer.setAborted(executeResult);
    return;
  }
}

// void CoverMap::executeDock()
// {
//   // make sure we have an idea of where the dock is
//   if(!robot.hasDockLocation())
//   {
//     robot.stop();
//     executeResult.text = "Docking failed because we have no idea where the dock is.";
//     executeServer.setAborted(executeResult);
//     return;
//   }
//   else
//     ROS_INFO("CoverMap::executeDock(): Robot has dock location");

//   // Initialize robot
//   if(!robot.enableMotors())
//   {
//     robot.stop();
//     executeResult.text = "Docking failed because we could not enable motors.";
//     executeServer.setAborted(executeResult);
//     return;
//   }
//   else
//     ROS_INFO("CoverMap::executeDock(): Motors successfully enabled.");
  
//   // Attempt to move to dock approach location
//   geometry_msgs::PoseStamped farApproach = dockApproach;
//   farApproach.header.stamp = ros::Time::now();
//   farApproach.pose.position.x += 0.3;
//   if(!asyncNavGoal(farApproach,0.1))
//   {
//     executeResult.text = "Robot failed to move to dock location.";
//     executeServer.setAborted(executeResult);
//     return;
//   }
//   else
//     ROS_INFO("CoverMap::executeDock(): Successfully moved to dock approach location.");
  
//   // turn towards dock
//   dockLocation.header.stamp = ros::Time::now();
//   if(!asyncVelGoal(dockLocation,0.1,true,false))
//   {
//     // do nothing. we might still be able to dock without having done this move
//     ROS_ERROR("CoverMap: Failed to turn to dock location");
//   }
//   else
//     ROS_INFO("CoverMap::executeDock(): Successfully turned towards dock.");

//   // tell dock_locator to update dock location
//   if(!robot.findDockLocation())
//   {
//     executeResult.text = "Robot failed to find dock in laser scan.";
//     executeServer.setAborted(executeResult);
//     return;
//   }
//   else
//     ROS_INFO("CoverMap::executeDock(): Dock locator updated dock location.");

//   // Sleep to let dock frame update
//   ros::Duration(0.1).sleep();

//   // Attempt to move to updated dock approach location
//   dockApproach.header.stamp = ros::Time::now();
//   if(!asyncNavGoal(dockApproach,0.1))
//   {
//     ROS_ERROR("CoverMap: Failed nav goal to dock location. Trying again");
//     dockApproach.header.stamp = ros::Time::now();
//     asyncNavGoal(dockApproach,0.1);
//   }
//   else
//     ROS_INFO("CoverMap::executeDock(): Successfully moved to updated dock approach location.");

//   // now use a velocity goal to be more accurate
//   dockApproach.header.stamp = ros::Time::now();
//   if(!asyncVelGoal(dockApproach,0.04,false,false,true))
//   {
//     executeResult.text = "Robot failed to move to precise dock location.";
//     executeServer.setAborted(executeResult);
//     return;
//   }
//   else
//     ROS_INFO("CoverMap::executeDock(): Successfully moved (vel goal) to precise dock approach location.");

//   // turn towards dock
//   dockLocation.header.stamp = ros::Time::now();
//   if(!asyncVelGoal(dockLocation,0.1,true,false))
//   {
//     // do nothing. we might still be able to dock without having done this move
//     ROS_ERROR("CoverMap: Failed to turn to dock location");
//   }
//   else
//     ROS_INFO("CoverMap::executeDock(): Successfully turned towards dock.");

//   // update the dock location one more time
//   if(!robot.findDockLocation())
//   {
//     executeResult.text = "Robot failed to find final dock position in laser scan.";
//     executeServer.setAborted(executeResult);
//     return;
//   }
//   else
//     ROS_INFO("CoverMap::executeDock(): Dock locator updated dock location.");

//   // Sleep to let dock frame update
//   ros::Duration(0.1).sleep();

//   // use velocity goal to move to final dock approach
//   dockApproach.header.stamp = ros::Time::now();
//   if(!asyncVelGoal(dockApproach,0.04,false,false,true))
//   {
//     executeResult.text = "Robot failed to move to final precise dock location.";
//     executeServer.setAborted(executeResult);
//     return;
//   }
//   else
//     ROS_INFO("CoverMap::executeDock(): Successfully moved to final precise dock approach location.");

//   // turn away from dock
//   geometry_msgs::PoseStamped awayFromDock = dockLocation;
//   awayFromDock.header.stamp = ros::Time::now();
//   awayFromDock.pose.position.x = 3.0;
//   if(!asyncVelGoal(awayFromDock,0.01,true,false))
//   {
//     executeResult.text = "Robot failed to turn away from dock.";
//     executeServer.setAborted(executeResult);
//     return;
//   }
//   else
//     ROS_INFO("CoverMap::executeDock(): Successfully turned away from dock.");

//   // back on to dock
//   if(asyncDockReverse(0.5, 0.03))
//   {
//     // set current location as dock location
//     robot.setDockLocation();

//     executeResult.text = "Robot docked successfully";
//     executeServer.setSucceeded(executeResult);
//     return;
//   }
//   else
//   {
//     executeResult.text = "Robot failed to move backwards onto dock.";
//     executeServer.setAborted(executeResult);
//     return;
//   }
// }

// void CoverMap::executeUndock()
// {
//   // Initialize robot
//   if(!robot.enableMotors())
//   {
//     robot.stop();
//     executeResult.text = "Undocking failed because we could not enable motors.";
//     executeServer.setAborted(executeResult);
//     return;
//   }

//   // Get current location and add to x to move forward
//   //geometry_msgs::PoseStamped fPose = robot.getRobotPose();
//   //if(!transformPose(fPose,"base_link"))
//   //{
//   //  executeResult.text = "Robot failed to undock because of TF error.";
//   //  executeServer.setAborted(executeResult);
//   //  return;
//   //}
//   //fPose.pose.position.x += dockApproach.pose.position.x;

//   // Move forward off of dock (async)
//   dockApproach.header.stamp = ros::Time::now();
//   if(asyncVelGoal(dockApproach, 0.1, false, true, true))
//   {
//     executeResult.text = "Robot succesfully undocked";
//     executeServer.setSucceeded(executeResult);
//     return;
//   }
//   else
//   {
//     // If we moved far enough off the dock, set to success anyway
//     if(robot.getNavGoalDistance(true) < dockApproach.pose.position.x/2.0)
//     {
//       executeResult.text = "Robot succesfully undocked, but did not move all the way to dockApproach position";
//       executeServer.setSucceeded(executeResult);
//       return;
//     }
//     else
//     {
//       executeResult.text = "Robot failed to move forward off of dock.";
//       executeServer.setAborted(executeResult);
//       return;
//     }
//   }
// }

bool CoverMap::checkPlanVars()
{
  // If the last plan is invalid or finished, do not continue to execution
  if(plan.size() < 1 || planPos == plan.end())
  {
    executeResult.text = "Cannot resume plan. We have invalid plan or iterator stored.";
    executeServer.setAborted(executeResult);
    return false;
  }
  else
    return true;
}

bool CoverMap::initializePlanVars(const full_coverage::ExecuteGoalConstPtr& goal)
{
  if(goal->plan.poses.size()<=0)
  {
    executeResult.text = "Cannot start plan. Passed plan contains no poses.";
    executeServer.setAborted(executeResult);
    return false;
  }
  else if(goal->plan.poses.size() != goal->plan.allow_skip.size())
  {
    executeResult.text = "Cannot start plan. Passed plan info sizes don't match.";
    executeServer.setAborted(executeResult);
    return false;
  }
  else
  {
    cellWidth = goal->plan.cell_width * 1.1;
    plan.clear();
    for(int i = 0; i < goal->plan.poses.size(); i++)
    {
      plan.push_back(std::make_pair(goal->plan.poses[i],goal->plan.allow_skip[i]));
    }
    planPos = plan.begin();
    skipPos = plan.begin();
    skipResetPos = plan.begin();
    return true;
  }
}

fcPlan::iterator CoverMap::getProgressPos(fcPlan::iterator skipP, fcPlan::iterator planP)
{
  geometry_msgs::PoseStamped rPose = robot.getRobotPose();
  bool xPositive = true;
  bool yPositive = true;
  bool xNeutral = false;
  bool yNeutral = false;
  int dist = std::distance(skipP,planP);
  if(dist == 1) // if planP is the only pose after skipP (and we didn't make it to planP), then we want to go to skipP
    return skipP;
  if(dist > 0)
  {
    // convert robot pose if necessary
    if(!transformPose(rPose,(*skipP).first.header.frame_id))
    {
      ROS_ERROR("CoverMap: getProgressPos(): Could not get progress. Pose transform failed");
      return skipP;
    }

    // check which direction we were trying to go
    double tol = 0.001;
    double xDiff = (*(skipP+1)).first.pose.position.x - (*skipP).first.pose.position.x;
    double yDiff = (*(skipP+1)).first.pose.position.y - (*skipP).first.pose.position.y;
    double xDiff2 = (*(skipP+2)).first.pose.position.x - (*skipP).first.pose.position.x;
    double yDiff2 = (*(skipP+2)).first.pose.position.y - (*skipP).first.pose.position.y;
    if((fabs(xDiff) < tol || xDiff < 0.0) && (fabs(xDiff2) < tol || xDiff2 < 0.0))
      xPositive = false;
    if((fabs(yDiff) < tol || yDiff < 0.0) && (fabs(yDiff2) < tol || yDiff2 < 0.0))
      yPositive = false;
    if(fabs(xDiff) < tol && fabs(xDiff2) < tol)
      xNeutral = true;
    if(fabs(yDiff) < tol && fabs(yDiff2) < tol)
      yNeutral = true;

    // if final pose is not in same direction as initial increment, return initial skip position
    xDiff = (*planP).first.pose.position.x - (*skipP).first.pose.position.x;
    yDiff = (*planP).first.pose.position.y - (*skipP).first.pose.position.y;
    if( fabs(xDiff) > tol && (xPositive && xDiff < 0.0)  ||
        fabs(xDiff) > tol && (!xPositive && xDiff > 0.0) ||
        fabs(yDiff) > tol && (yPositive && yDiff < 0.0)  ||
        fabs(yDiff) > tol && (!yPositive && yDiff > 0.0) )
    {
      ROS_ERROR("CoverMap: getProgressPos(): Iterators were not in line. Returning skip position.");
      return skipP;
    }

    // find how far we went
    while(skipP != planP)
    {
      xDiff = rPose.pose.position.x - (*skipP).first.pose.position.x;
      yDiff = rPose.pose.position.y - (*skipP).first.pose.position.y;
      if(!xNeutral)
      {
        if(fabs(xDiff) > tol && (xPositive && xDiff < 0.0))
          return skipP;
        if(fabs(xDiff) > tol && (!xPositive && xDiff > 0.0))
          return skipP;
      }
      if(!yNeutral)
      {
        if(fabs(yDiff) > tol && (yPositive && yDiff < 0.0))
          return skipP;
        if(fabs(yDiff) > tol && (!yPositive && yDiff > 0.0))
          return skipP;
      }
      skipP = skipP + 1;
    }
    return skipP;
  }
  else
  {
    ROS_ERROR("CoverMap: getProgressPos(): plan position was not ahead of skip position. This should not happen. Returning skip position.");
    return skipP;
  }
}

bool CoverMap::toSkip(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2, geometry_msgs::PoseStamped pose3, geometry_msgs::PoseStamped zagRef)
{
  bool skip = false;

  // We shouldn't be skipping poses if we are not in velocity command mode
  if(CMD_POSE == cmdMode)
    return skip;

  // if all three poses are in a vertical line
  if(!lastZag && inLineX(pose1,pose2) && inLineX(pose2,pose3))
  {
    // if poses are not reversing vertical direction
    if((pose1.pose.position.y >= pose2.pose.position.y && pose2.pose.position.y >= pose3.pose.position.y) ||
       (pose1.pose.position.y <= pose2.pose.position.y && pose2.pose.position.y <= pose3.pose.position.y))
    {
      skip = true;
      lastLine = true;
    }
  }
  // if all three poses are in a horizontal line
  if(!skip && !lastZag && inLineY(pose1,pose2) && inLineY(pose2,pose3))
  {
    // if poses are not reversing horizontal direction
    if((pose1.pose.position.x >= pose2.pose.position.x && pose2.pose.position.x >= pose3.pose.position.x) ||
       (pose1.pose.position.x <= pose2.pose.position.x && pose2.pose.position.x <= pose3.pose.position.x))
    {
      skip = true;
      lastLine = true;
    }
  }
  bool n1 = nextTo(pose1,pose2);
  bool n2 = nextTo(pose2,pose3);
  // if poses are zig zagging one coarse cell at a time
  if(!skip && !lastLine && nextTo(pose1,pose2) && nextTo(pose2,pose3)) // check if we are in a string of 3 that have all moved only one cell
  {
    if(!(inLineX(zagRef,pose3) || inLineY(zagRef,pose3) || inLineX(pose1,pose3) || inLineY(pose1,pose3))) // make sure zig zag instead of zig zig
    {
      skip = true;
      lastZag = true;
    }
  }

  if(!skip)
  {
    lastLine = false;
    lastZag = false;
  }

  return skip;
}

bool CoverMap::nextTo(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2)
{
  if(sqrt(pow(pose1.pose.position.x-pose2.pose.position.x,2) + pow(pose1.pose.position.y-pose2.pose.position.y,2)) < cellWidth)
    return true;
  else
    return false;
}

bool CoverMap::inLineX(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2)
{
  // Check if the two poses have the same x position
  if(fabs(pose1.pose.position.x - pose2.pose.position.x) < 0.05)
    return true;
  else
    return false;
} // inLineX

bool CoverMap::inLineY(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2)
{
  // Check if the two poses have the same y position
  if(fabs(pose1.pose.position.y - pose2.pose.position.y) < 0.05)
    return true;
  else
    return false;
} //inLineY

bool CoverMap::checkPose(geometry_msgs::PoseStamped pose)
{
  if(!gotMap)
  {
    ROS_ERROR_THROTTLE(1.0,"Cover map does not have occupancy grid and cannot check poses.");
    return true;
  }

  // convert map pose if necessary
  if(!transformPose(pose, map.header.frame_id))
  {
    ROS_ERROR("CoverMap: checkPose() failed to convert pose to map frame");
  }

  // convert pose to map coordinates
  double wx = pose.pose.position.x;
  double wy = pose.pose.position.y;
  double ox = map.info.origin.position.x;
  double oy = map.info.origin.position.y;
  if(wx < ox || wy < oy)
  {
    ROS_ERROR("CoverMap: checkPose() failed: pose position less than origin.");
    return true;
  }

  double mx = (int)((wx-ox)/map.info.resolution);
  double my = (int)((wy-oy)/map.info.resolution);

  if(mx >= map.info.width || my >= map.info.height)
  {
    ROS_ERROR("CoverMap: checkPose() failed: pose position greater than map bounds.");
    return true;
  }

  double index = my*map.info.width + mx;

  if(index < 0 || index >= map.data.size())
  {
    ROS_ERROR("CoverMap: checkPose() failed: index greater than map bounds.");
    return true;
  }

  if(map.data[index] < 50)
    return true;
  else
    return false;

}

bool CoverMap::transformPose(geometry_msgs::PoseStamped &pose, std::string to_frame)
{
  // convert pose if necessary
  if(pose.header.frame_id.compare(to_frame) !=0)
  {
    tf::Stamped<tf::Pose> in, out;
    pose.header.stamp = ros::Time(0);
    tf::poseStampedMsgToTF(pose, in);
    try
    {
      if(listener->waitForTransform(to_frame, pose.header.frame_id, ros::Time(0), ros::Duration(1.0)))
      {
        listener->transformPose(to_frame, in, out);
        tf::poseStampedTFToMsg(out, pose);
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
  return true;
}

bool CoverMap::goToPoint(geometry_msgs::PoseStamped nav_pose)
{
  nav_pose.header.stamp = ros::Time::now();

  if(CMD_VEL == cmdMode)
    return asyncVelGoal(nav_pose, 0.1);
  else
    return asyncNavGoal(nav_pose, 0.1);
} // goToPoint

bool CoverMap::asyncNavGoal(geometry_msgs::PoseStamped pose, double tol)
{
  // If we are in navigation command mode, send nav goal to robot
  if(!robot.sendNavGoal(pose, false))
  {
    ROS_ERROR("CoverMap: Failed to send nav goal");
    return false;
  }
  // Wait for goal to complete
  ros::Rate attemptRate(10.0);
  while(robot.getNavGoalStatus() == robot_interface::GoalStatus::ACTIVE)
  {
    if(robot.getNavGoalDistance(true) <= tol || executeServer.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("Sending cancel nav goal");
      robot.cancelNavGoal();
    }
    attemptRate.sleep();
  }

  // Return success/failure
  if(robot.getNavGoalStatus() == robot_interface::GoalStatus::SUCCESS || robot.getNavGoalDistance(true) <= tol)
    return true;
  else
    return false;
}

bool CoverMap::asyncVelGoal(geometry_msgs::PoseStamped pose, double tol, bool justTurn, bool justDrive, bool justLaser)
{
  // If we are in velocity command mode, send velocity goal
  std::future<bool> velSuccess = std::async(std::launch::async, &robot_interface::RobotClient::sendVelGoal, &robot, pose, tol, justTurn, justDrive, justLaser);
  // sleep so vel active flag is set
  ros::Duration(0.2).sleep();

  // Wait for goal to complete
  ros::Rate attemptRate(10.0);
  while(robot.isVelGoalActive())
  {
    if(robot.getNavGoalDistance(true) <= tol || executeServer.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("Sending cancel vel goal");
      robot.cancelVelGoal();
    }
    attemptRate.sleep();
  }

  // return success/failure
  return velSuccess.get();
}

// bool CoverMap::asyncDockReverse(double desired, double tol)
// {
//   // If we are in velocity command mode, send velocity goal
//   std::future<bool> velSuccess = std::async(std::launch::async, &robot_interface::RobotClient::dockReverse, &robot, desired, tol);
//   // sleep so vel active flag is set
//   ros::Duration(0.2).sleep();

//   // Wait for goal to complete
//   ros::Rate attemptRate(10.0);
//   while(robot.isVelGoalActive())
//   {
//     if(executeServer.isPreemptRequested() || !ros::ok())
//     {
//       ROS_INFO("Sending cancel vel goal");
//       robot.cancelVelGoal();
//     }
//     attemptRate.sleep();
//   }

//   // return success/failure
//   return velSuccess.get();
// }

} // namespace full_coverage

int main(int argc, char **argv) {
  // ROS setup
  ros::init(argc, argv, "cover_map");

  // Create sweeper instance of CoverMap
  full_coverage::CoverMap* sweeper = new full_coverage::CoverMap();
  
  // Wait for shutdown so callbacks are still active
  ros::waitForShutdown();

  ROS_INFO("Cover map shutting down.");
  return 0; // success
} // main
