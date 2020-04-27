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

#include "wall_planner.h"

namespace full_coverage
{

WallPlanner::WallPlanner() :
  startIndex(0),
  isInitialized(false)
{
  // Do nothing
}

WallPlanner::~WallPlanner()
{
  // Do nothing
}

bool WallPlanner::initialize(CellGrid map, int ot, int cd, int nbc)
{
  // Initialize wave cells
  if(!coarseCells.initialize(map, ot, nbc))
  {
    ROS_ERROR("WallPlanner: Wave cells failed to initalize");
    return false;
  }
  
  ROS_INFO("WallPlanner: Wave cells initialized");
  
  // Initialize robot cells
  if(!robotCells.initialize(&coarseCells, cd))
  {
    ROS_ERROR("WallPlanner: Robot cells failed to initalize");
    return false;
  }
  
  ROS_INFO("WallPlanner: Robot cells initalized");

  // Set init flag
  isInitialized = true;
  
  return true;
}

bool WallPlanner::newMap(CellGrid map, int ot, int cd, int nbc, std::string poseFrame, std::string lastFrame)
{
  // Copy over old grids
  CoarseGrid oldCoarse(coarseCells);

  // Create new grid objects
  coarseCells = CoarseGrid();
  robotCells = RobotGrid();

  // Initialize new objects with new data
  if(!initialize(map, ot, cd, nbc))
    return false;

  // Cycle through new map and fill it with visited data from old map
  for(int i = 0; i<coarseCells.getSize(); i++)
  {
    CellValue cv;
    if(coarseCells.getValue(i, cv) && (OPEN_CELL == cv || BORDER_CELL == cv))
    {
      geometry_msgs::Pose newPose;
      if(coarseCells.getPose(i,newPose))
      {
        // If we are in a new frame, transform the new pose to the old frame
        bool skipPose = false;
        geometry_msgs::PoseStamped oldPoseStamped, newPoseStamped;
        newPoseStamped.pose = newPose;
        newPoseStamped.header.frame_id = poseFrame;
        newPoseStamped.header.stamp = ros::Time(0);
        if(poseFrame.compare(lastFrame) != 0)
        {
          try
          {
            tf::Stamped<tf::Pose> in, out;
            tf::poseStampedMsgToTF(newPoseStamped,in);
            if(listener.waitForTransform(lastFrame, poseFrame, ros::Time(0), ros::Duration(1.0)))
            {
              listener.transformPose(lastFrame, in, out);
              tf::poseStampedTFToMsg(out, oldPoseStamped);
            }
            else
            {
              ROS_ERROR_THROTTLE(1.0, "WallPlanner: Transform from old grid to new timed out.");
              skipPose = true;
            }
          }
          catch(...)
          {
            ROS_ERROR_THROTTLE(1.0, "WallPlanner: Exception caught in transform from old grid to new.");
            skipPose = true;
          }
        }
        else
        {
          oldPoseStamped = newPoseStamped;
        }
        if(!skipPose)
        {
          // if the old cell was visited, visit the new one
          bool visited = false;
          bool attempted = false;
          int oldIndex = oldCoarse.getIndex(oldPoseStamped.pose);
          CellValue ov;
          if(oldIndex < 0 || !oldCoarse.isVisited(oldIndex, visited) || !oldCoarse.isAttempted(oldIndex, attempted))
          {
            ROS_ERROR_THROTTLE(1.0, "WallPlanner: failed to check if cell in old grid was visited.");
          }
          else if(visited)
          {
            coarseCells.visit(i);
          }
          else if(attempted)
          {
            coarseCells.setAttempted(i);
          }
          // if the old cell wasn't open, and it didn't contain visited data, we need to take it from a neighbor
          else if(oldCoarse.getValue(oldIndex, ov) && OPEN_CELL != ov && OPEN_CELL == cv)
          {
            int r,c;
            if(oldCoarse.getCoordinate(oldIndex,r,c))
            {
              int nIndex;
              if(oldCoarse.getIndex(nIndex,r+1,c) && oldCoarse.isVisited(nIndex, visited) && visited)
              {
                coarseCells.visit(i);
              }
              else if(oldCoarse.getIndex(nIndex,r-1,c) && oldCoarse.isVisited(nIndex, visited) && visited)
              {
                coarseCells.visit(i);
              }
              else if(oldCoarse.getIndex(nIndex,r,c+1) && oldCoarse.isVisited(nIndex, visited) && visited)
              {
                coarseCells.visit(i);
              }
              else if(oldCoarse.getIndex(nIndex,r,c-1) && oldCoarse.isVisited(nIndex, visited) && visited)
              {
                coarseCells.visit(i);
              }
            }
            else
            {
              ROS_ERROR_THROTTLE(1.0, "WallPlanner: failed to get r,c in old map.");
            }
          }
        }
      }
      else
      {
        ROS_ERROR_THROTTLE(1.0, "WallPlanner: Could not get pose of cell in new coarse grid.");
      }
    }
  }
  // Now we patch the edges of the map. This is done because false non-visited spots can crop up around borders
  // due to the translation of the old map to the new map
  // not sure if we actually want to do this yet..
  //patchCoarseGrid();

  return true;
}

void WallPlanner::patchCoarseGrid()
{
  // Cycle through new map and check for unvisited open space next to boarders
  std::vector<int> indices;
  for(int i = 0; i<coarseCells.getSize(); i++)
  {
    CellValue cv;
    bool isVis = false;
    if(coarseCells.getValue(i, cv) && coarseCells.isVisited(i,isVis) && OPEN_CELL == cv && !isVis)
    {
      std::vector<CoverageCell*> nbs;
      std::vector<CoverageCell*> secondNbs;
      int borders = 0;
      int visited = 0;
      if(coarseCells.getNeighbors(i,nbs))
      {
        for(int k = 0; k < nbs.size(); k++)
        {
          if(BORDER_CELL == nbs[k]->getValue())
          {
            borders++;
          }
          else if(OPEN_CELL == nbs[k]->getValue())
          {
            std::vector<CoverageCell*> tempNbs;
            nbs[k]->getNeighbors(tempNbs);
            secondNbs.insert(secondNbs.end(),tempNbs.begin(),tempNbs.end());
          }
          if(nbs[k]->isVisited())
          {
            visited++;
          }
        }
      }
      for(int k = 0; k < secondNbs.size(); k++)
      {
        if(BORDER_CELL == nbs[k]->getValue())
        {
          borders++;
        }
        if(nbs[k]->isVisited())
        {
          visited++;
        }
      }
      // if a lot of the neighbors are visited and a lot are border cells, mark as visited
      if(borders > 2 && visited > 2)
        indices.push_back(i);
    }
  }
  for(int i=0; i < indices.size(); i++)
    coarseCells.visit(indices[i]);
}

bool WallPlanner::fill(geometry_msgs::Pose startPose)
{
  // Set start state if it is valid
  if(!setStartIndex(startPose))
  {
    return false;
  }

  // get coarse cell start index and use it to find reachable cells
  int coarseIndex = startIndex;
  if(robotCells.getCoarseIndex(coarseIndex))
  {
    if(!coarseCells.findReachable(coarseIndex))
    {
      return false;
    }
  }

  // Update robot cells aggragate values now that coarse cells have changed
  for(int i = 0; i<robotCells.getSize(); i++)
    robotCells.updateValue(i);

  return true;
}

bool WallPlanner::replanFromPoses(std::vector<geometry_msgs::Pose> &poses)
{
  robotCells.getUnvisitedCells(poses);

  return poses.size()>0;
}

bool WallPlanner::getPlan(std::vector<geometry_msgs::Pose> &plan, std::vector<bool> &planInfo, bool secAtt)
{
  // Copy off second attempt variable
  robotCells.setSecondAttempt(secAtt);

  // Clear the plan
  planInfo.clear();
  plan.clear();

  // Use debugPoses vector to print out map of plan order
  std::vector<int> debugPoses;

  // set the index to start position and initial direction as north
  int index = startIndex;
  MapDirection dir = robotCells.getWallDirection(index);
  if(index < 0)
    return false;
  
  // Plan order of poses
  bool done = false;
  int numPlanned = addPose(index, dir, plan, planInfo);   // Add the start pose as the first pose in the plan
  debugPoses.push_back(index);          // Make the start pose the first pose in debug_poses
  int stuckCount = 0;
  int newIndex = 0;
  bool lastHall = false;

  // First, head North until we find a wall in front of us or on our left
  MapDirection newDir = MAP_NORTH;
  bool wallFound = false;
  while(!wallFound && stuckCount<=4) // Exit if we find a wall or are stuck
  {
    if(robotCells.findWall(index, dir, newIndex, newDir, wallFound))  // We can only move to a neighbor if it is valid and unplanned
    {
      if(index == newIndex)
      {
        dir = newDir;
        stuckCount++;
      }
      else
      {
        stuckCount = 0;
        index = newIndex; // set the index to the neighbor we will move to
        dir = newDir;
        //numPlanned += addPose(index, dir, plan);            // don't add poses while we are looking for a wall to the plan
        debugPoses.push_back(index);
      }
    }
    else // Our start index was invalid
    {
      ROS_ERROR("Failed to find wall. Cannot complete plan.");
      done = true;
      wallFound = true;
    }
  }
  numPlanned += addPose(index, dir, plan, planInfo);
  stuckCount = 0;
  while(stuckCount<=2 && !done)  // if we get stuck, we exit the loop and use the hop function to find the closest unplanned cell
  {
    if(!lastHall && stuckCount == 2) // special case where we have done a 180 because we are stuck in a hall the size of the robot
    {
      if(robotCells.leaveHall(index, dir, newIndex, newDir))
      {
        index = newIndex;
        dir = newDir;
        stuckCount = 0;
        lastHall = true;
        numPlanned += addPose(index, dir, plan, planInfo);            // add the new cell to the plan
        debugPoses.push_back(index);
      }
      else
      {
        stuckCount++;
      }
    }
    else if(lastHall && stuckCount == 2)
    {
      stuckCount++;
    }
    if(stuckCount<=2)
    {
      if(robotCells.getBorderNeighbor(index, dir, newIndex, newDir))  // We can only move to a neighbor if it is valid and unplanned
      {
        if(index == newIndex)
        {
          dir = newDir;
          stuckCount++;
        }
        else
        {
          lastHall = false;
          index = newIndex; // set the index to the neighbor we will move to
          dir = newDir;
          stuckCount = 0;
          numPlanned += addPose(index, dir, plan, planInfo, true);            // add the new cell to the plan
          debugPoses.push_back(index);
        }
      }
      else // if all valid neighbors have been planned, we are stuck
      {
        //ROS_INFO("Stuck: no neighbors are unplanned.");
        stuckCount = 3;
      }
    }
    //if(debugPoses.size()%20==0)
      //printPlan(debugPoses);
  }

  // if we didn't find a valid plan, check if there are any reachable unknown cells and go to one
  if(numPlanned <= (robotCells.getWidth() * robotCells.getHeight()))
  {
    newIndex = index;
    robotCells.findOpenNeighbor(newIndex, 0.0, true);
    if(index != newIndex)
      numPlanned += addPose(index, dir, plan, planInfo);
  }
    
  ROS_INFO_STREAM("WallPlanner: Planned " << numPlanned << " coarse cells. Plan size is: " << plan.size());
  //printPlan(debugPoses);

  return true;
}

void WallPlanner::printPlan(std::vector<int> poses)
{
  /**************************************************************************************************/
  /** This code block will print the order of the coverage plan. Use only for debugging purposes. **/
  /**************************************************************************************************/
  int grid_cols = coarseCells.getNumCols();
  int grid_rows = coarseCells.getNumRows();
  
  // Pretty colors to distinguish walls from free space
  std::string color_red = "\033[0;31m";
  std::string color_green = "\033[0;32m";
  std::string color_blue = "\033[0;34m";
  std::string default_col = "\033[0m";
  std::string color = default_col;
  
  std::cout << "-----------------------------------------------------------------" << std::endl;
  int half_rw = floor(robotCells.getWidth()/2.0);
  int half_rh = floor(robotCells.getHeight()/2.0);
  for (int row = 0; row < grid_rows; row++){
    std::cout << "|";
    for (int col = 0; col < grid_cols; col++){
      int index = 0;
      coarseCells.getIndex(index, row, col); // Finding the appropriate cell in vectorized form
      CellValue val = UNKNOWN_CELL;
      if (coarseCells.getValue(index,val) && val == OBSTRUCTED_CELL) // If the cell is an obstacle
      {
        std::cout<<color_red<< "|   |";
      }
      else if (coarseCells.getValue(index,val) && val == BORDER_CELL) // If the cell is an obstacle
      {
        std::cout<<color_blue<< "|   |";
      }
      else
      {
        bool vis = false;
        if(coarseCells.isPlanned(index,vis) && vis)
          color = color_green;
        else
          color = default_col;
        int order = 0;
        int rIndex = 0;
        if(robotCells.getIndex(rIndex,row-half_rh,col-half_rw))
        {
          std::vector<int>::iterator it = std::find(poses.begin(),poses.end(),rIndex);
          if(it == poses.end())
            order = -1;
          else
            order = std::distance(poses.begin(), it);
        }
        else
          order = -1;
        
        if(order<0)
          std::cout<< color << "|   |"; 
        else if(order<10)
          std::cout<< color << "|  "<< default_col <<order << color <<"|"; 
        else if(order<100)
          std::cout<<  color <<"| "<< default_col <<order << color <<"|"; 
        else if(order<1000)
          std::cout<< color << "|"<< default_col <<order << color <<"|"; 
        //else
        //  std::cout<< "|"<< color_red <<order << default_col <<"|"; 
      }
    }
    std::cout << std::endl;
  }
  std::cout << "----------------------------------------------------------------" << std::endl;
}

bool WallPlanner::setStartIndex(geometry_msgs::Pose startPose)
{
  // Find the cell that contains the start index
  int cellIndex = robotCells.getIndex(startPose);
  
  // Make sure the cell is valid and not obstructed
  CellValue val = UNKNOWN_CELL;
  bool vis = false;
  if(robotCells.getValue(cellIndex, val) && robotCells.isVisited(cellIndex, vis))
  {
    // If the start cell is not open or it is visited, we move the start position to the closest open cell
    if(OPEN_CELL != val || vis)
    {
      int oldIndex = cellIndex;
      bool nbr = robotCells.findOpenNeighbor(cellIndex);
      if(!nbr || cellIndex <= 0 || cellIndex >= robotCells.getSize() || oldIndex == cellIndex)
      {
        cellIndex = oldIndex;
        nbr = robotCells.findOpenNeighbor(cellIndex, 0.35);
        if(!nbr || cellIndex <= 0 || cellIndex >= robotCells.getSize() || oldIndex == cellIndex)
        {
          ROS_ERROR("WallPlanner: Start pose invalid: Could not find empty neighbor.");
          ROS_ERROR_STREAM("Start pose x: " << startPose.position.x << ". y: " << startPose.position.y);
          return false;
        }
      }
    }
    startIndex = cellIndex;
  }
  else
  {
    ROS_ERROR("WallPlanner: Start pose invalid: not within map bounds.");
    ROS_ERROR_STREAM("Start pose x: " << startPose.position.x << ". y: " << startPose.position.y);
    CoverageCell* temp1;
    CoverageCell* temp2;
    robotCells.getCell(0,temp1);
    robotCells.getCell(robotCells.getSize()-1, temp2);
    ROS_ERROR_STREAM("WallPlanner: Map bounds x: " << temp1->getPose().position.x << " to " << temp2->getPose().position.x);
    ROS_ERROR_STREAM("WallPlanner: Map bounds y: " << temp1->getPose().position.y << " to " << temp2->getPose().position.y);
    return false;
  }
  
  return true;
}

int WallPlanner::addPose(int i, MapDirection dir, std::vector<geometry_msgs::Pose>& plan, std::vector<bool> &planInfo, bool info)
{
  // Get the center pose of the cell to be added to the plan
  geometry_msgs::Pose pose;
  robotCells.getPose(i,pose);
  
  // Determine which direction the robot will be coming so we know what orientation it should stop in
  tf::Quaternion quat;
  switch(dir)
  {
    case MAP_SOUTH:
      quat.setRPY(0,0,-1.57);
      break;
    case MAP_WEST:
      quat.setRPY(0,0,3.14);
      break;
    case MAP_NORTH:
      quat.setRPY(0,0,1.57);
      break;
    default:
      quat.setRPY(0,0,0);
      break;
  }
  tf::quaternionTFToMsg(quat,pose.orientation);
  
  // add the pose to the plan
  plan.push_back(pose);
  planInfo.push_back(info);
  
  // tell the cell it has been planned
  int numPlanned = 0;
  robotCells.setPlanned(i, numPlanned);
  return numPlanned;
}

void WallPlanner::visit(geometry_msgs::Pose robotPose)
{
  robotCells.visit(robotPose);
}

void WallPlanner::setAttempted(std::vector<geometry_msgs::Pose> robotPose)
{
  for(int i = 0; i< robotPose.size(); i++)
  {
    int ind = robotCells.getIndex(robotPose[i]);
    if(ind>=0)
    {
      bool att,vis;
      robotCells.isAttempted(ind,att);
      robotCells.isVisited(ind,vis);
      if(!att)
        robotCells.setAttempted(ind);
      else
        robotCells.visit(ind);
    }
    else
      ROS_ERROR("WallPlanner: told to attempt a cell with invalid index.");
  }
}

bool WallPlanner::initialized()
{
  return isInitialized;
}

double WallPlanner::getCellWidth()
{
  if(!isInitialized)
  {
    ROS_ERROR("WallPlanner: getCellWidth called on uninitialized wall planner");
    return 0.0;
  }

  return coarseCells.getCellWidth();
}

full_coverage::DebugGrid WallPlanner::getDebugMsg()
{
  full_coverage::DebugGrid msg;
  msg.coarse_cells.clear();
  for(int i = 0; i<coarseCells.getSize(); i++)
  {
    full_coverage::DebugCell cmsg;
    std::vector<geometry_msgs::Pose> corners;
    coarseCells.getCorners(i,corners);
    double xl = 0.0;
    double xr = 0.0;
    double yl = 0.0;
    double yh = 0.0;
    if(corners.size()>0)
    {
      xl = corners[0].position.x;
      xr = corners[0].position.x;
      yl = corners[0].position.y;
      yh = corners[0].position.y;
      for(int j = 1; j<corners.size(); j++)
      {
        if(corners[j].position.x < xl)
          xl = corners[j].position.x;
        if(corners[j].position.x > xr)
          xr = corners[j].position.x;
        if(corners[j].position.y < yl)
          yl = corners[j].position.y;
        if(corners[j].position.y > yh)
          yh = corners[j].position.y;
      }
    }
    cmsg.tl_vertex.position.x = xl;
    cmsg.tl_vertex.position.y = yh;
    cmsg.tl_vertex.position.z = 0.0;
    cmsg.tr_vertex.position.x = xr;
    cmsg.tr_vertex.position.y = yh;
    cmsg.tr_vertex.position.z = 0.0;
    cmsg.br_vertex.position.x = xr;
    cmsg.br_vertex.position.y = yl;
    cmsg.br_vertex.position.z = 0.0;
    cmsg.bl_vertex.position.x = xl;
    cmsg.bl_vertex.position.y = yl;
    cmsg.bl_vertex.position.z = 0.0;
    CellValue val;
    coarseCells.getValue(i,val);
    cmsg.cell_type = val;
    bool vis;
    coarseCells.isVisited(i,vis);
    bool att;
    coarseCells.isAttempted(i,att);
    if(vis)
      cmsg.cell_type = 10;
    else if(att)
      cmsg.cell_type = 11;
    msg.coarse_cells.push_back(cmsg);
  }
  msg.robot_cells.clear();
  for(int i = 0; i<robotCells.getSize(); i++)
  {
    full_coverage::DebugCell cmsg;
    std::vector<geometry_msgs::Pose> corners;
    robotCells.getCorners(i,corners);
    double xl = 0.0;
    double xr = 0.0;
    double yl = 0.0;
    double yh = 0.0;
    if(corners.size()>0)
    {
      xl = corners[0].position.x;
      xr = corners[0].position.x;
      yl = corners[0].position.y;
      yh = corners[0].position.y;
      for(int j = 1; j<corners.size(); j++)
      {
        if(corners[j].position.x < xl)
          xl = corners[j].position.x;
        if(corners[j].position.x > xr)
          xr = corners[j].position.x;
        if(corners[j].position.y < yl)
          yl = corners[j].position.y;
        if(corners[j].position.y > yh)
          yh = corners[j].position.y;
      }
    }
    cmsg.tl_vertex.position.x = xl;
    cmsg.tl_vertex.position.y = yh;
    cmsg.tl_vertex.position.z = 0.0;
    cmsg.br_vertex.position.x = xr;
    cmsg.br_vertex.position.y = yl;
    cmsg.br_vertex.position.z = 0.0;
    CellValue val;
    robotCells.getValue(i,val);
    cmsg.cell_type = val;
    msg.robot_cells.push_back(cmsg);
  }
  return msg;
}

} // namespace full_coverage

