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
* Description: Holds and calculates wavefront data for one cell of 
*  the map.
*********************************************************************/

#include "robot_cell.h"

namespace full_coverage
{

RobotCell::RobotCell() :
  CoverageCell(),
  inhabitants()
{
  // Do nothing
}

RobotCell::~RobotCell()
{
  // Do nothing
}

void RobotCell::initialize(int pos, std::vector<CoverageCell*> inh)
{
  // Initialize subclass variables
  inhabitants = inh;
  
  // Initialize super class
  CoverageCell::initialize(pos);
  
  if(inh.size()<=0)
  {
    ROS_ERROR("Robot Cell: Robot cell must have inhabitants");
    return;
  }
  
  isInitialized = true;
}

void RobotCell::setCorners()
{
  // Find vertices
  std::vector<geometry_msgs::Pose> verts;
  for(int i = 0; i<inhabitants.size(); i++)
  {
    std::vector<geometry_msgs::Pose> newVerts = inhabitants[i]->getCorners();
    for(int j = 0; j < newVerts.size(); j++)
      verts.push_back(newVerts[j]);
  }
  
  CoverageCell::setCorners(verts);
}

CellValue RobotCell::updateValue()
{
  // Make sure value and reachability is up to date
  bool obs = false;
  bool unk = false;
  bool bord = false;
  bool reach = true;
  for(int i = 0; i<inhabitants.size(); i++)
  {
    if(!inhabitants[i]->isReachable())
    {
      reach = false;
    }

    CellValue temp = inhabitants[i]->getValue();
    if(UNKNOWN_CELL == temp)
    {
      unk = true;
    }
    else if(BORDER_CELL == temp)
    {
      bord = true;
    }
    else if(OBSTRUCTED_CELL == temp)
    {
      obs = true;
    }
  }
  setReachable(reach);
  updateVisited();
  if(obs)
    CoverageCell::setValue(OBSTRUCTED_CELL);
  else if(bord)
    CoverageCell::setValue(BORDER_CELL);
  else if(unk)
    CoverageCell::setValue(UNKNOWN_CELL);
  else
    CoverageCell::setValue(OPEN_CELL);
  return CoverageCell::getValue();
}

bool RobotCell::setValue(CellValue val)
{
  for(int i = 0; i<inhabitants.size(); i++)
  {
    inhabitants[i]->setValue(val);
  }
  return CoverageCell::setValue(val);
}

void RobotCell::setPlanned(int &numChanged)
{
  numChanged = 0;
  for(int i = 0; i<inhabitants.size(); i++)
  {
    if(!inhabitants[i]->isPlanned())
    {
      inhabitants[i]->setPlanned();
      numChanged++;
    }
  }
  CoverageCell::setPlanned();
}

void RobotCell::visit()
{
  for(int i = 0; i<inhabitants.size(); i++)
  {
    inhabitants[i]->visit();
  }
  CoverageCell::visit();
}

int RobotCell::numPlanned()
{
  int num = 0;
  for(int i = 0; i<inhabitants.size(); i++)
  {
    if(inhabitants[i]->isPlanned())
      num++;
  }
}

bool RobotCell::isPlanned()
{
  updatePlanned();
  return CoverageCell::isPlanned();
}

bool RobotCell::isPlanned(int index)
{
  if(index >= 0 && index < inhabitants.size())
    return inhabitants[index]->isPlanned();
  else
  {
    ROS_ERROR("Robot Cell::isPlanned received invalid index.");
    return true;
  }
}

bool RobotCell::isPlanned(double percent)
{
  double pla = 0.0;
  for(int i = 0; i < inhabitants.size(); i++)
  {
    if(inhabitants[i]->isPlanned())
      pla += 1.0;
  }
  if(pla/inhabitants.size() <= percent)
    return false;
  else
    return true;
}

bool RobotCell::isVisited()
{
  updateVisited();
  return CoverageCell::isVisited();
}

bool RobotCell::isVisited(double percent)
{
  double vis = 0.0;
  for(int i = 0; i < inhabitants.size(); i++)
  {
    if(inhabitants[i]->isVisited())
      vis += 1.0;
  }
  if((vis/inhabitants.size()-0.0001) <= percent)
  {
    std::cout << "vis percent: " << vis/inhabitants.size()-0.0001 << std::endl;
    return false;
  }
  else
    return true;
}

bool RobotCell::isAttempted()
{
  updateAttempted();
  return CoverageCell::isAttempted();
}

bool RobotCell::isAttempted(double percent)
{
  double att = 0.0;
  for(int i = 0; i < inhabitants.size(); i++)
  {
    if(inhabitants[i]->isVisited() || inhabitants[i]->isAttempted())
      att += 1.0;
  }
  if((att/inhabitants.size()-0.0001) <= percent)
    return false;
  else
    return true;
}

bool RobotCell::isVisited(int index)
{
  if(index >= 0 && index < inhabitants.size())
    return inhabitants[index]->isVisited();
  else
  {
    ROS_ERROR("Robot Cell::isVisited received invalid index.");
    return true;
  }
}

bool RobotCell::isAttempted(int index)
{
  if(index >= 0 && index < inhabitants.size())
    return inhabitants[index]->isAttempted();
  else
  {
    ROS_ERROR("Robot Cell::isAttempted received invalid index.");
    return true;
  }
}

void RobotCell::updatePlanned()
{
  bool pla = true;
  for(int i = 0; i < inhabitants.size(); i++)
  {
    if(!inhabitants[i]->isPlanned())
      pla = false;
  }
  if(pla)
    CoverageCell::setPlanned();
}

void RobotCell::updateVisited()
{
  bool vis = true;
  for(int i = 0; i < inhabitants.size(); i++)
  {
    if(!inhabitants[i]->isVisited())
      vis = false;
  }
  if(vis)
    CoverageCell::visit();
}

void RobotCell::updateAttempted()
{
  bool att = true;
  for(int i = 0; i < inhabitants.size(); i++)
  {
    if(!inhabitants[i]->isVisited() && !inhabitants[i]->isAttempted())
      att = false;
  }
  if(att)
    CoverageCell::visit();
}

bool RobotCell::getCoarseIndex(int &ind)
{
  if(inhabitants.size()>0)
  {
    ind = inhabitants[0]->getPosition();
    return true;
  }
  else
  {
    ROS_ERROR("RobotCell: Cannot get coarse index. Cell has no inhabitants.");
    return false;
  }
}

} // namespace full_coverage

