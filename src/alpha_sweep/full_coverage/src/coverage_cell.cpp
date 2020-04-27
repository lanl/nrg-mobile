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

#include "coverage_cell.h"

namespace full_coverage
{

CoverageCell::CoverageCell() :
  position(-1),
  value(UNKNOWN_CELL),
  cellPlanned(false),
  cellVisited(false),
  cellAttempted(false),
  cellReachable(false),
  bounds(),
  isInitialized(false),
  neighbors()
{
  // Do nothing
}

CoverageCell::~CoverageCell()
{
  // Do nothing
}

void CoverageCell::initialize(int pos)
{
  // Make sure variables are valid
  if(pos<0)
  {
    ROS_ERROR("Coverage Cell: Position must be positive integer");
    return;
  }
  
  // Initialize variables
  position = pos;
  
  // Find vertices
  setCorners();
}

void CoverageCell::addNeighbor(CoverageCell* nb)
{
  neighbors.push_back(nb);
}

void CoverageCell::clearNeighbors()
{
  neighbors.clear();
}

void CoverageCell::getNeighbors(std::vector<CoverageCell*>& nbs)
{
  nbs = neighbors; 
}

void CoverageCell::getUnplannedNeighbors(std::vector<CoverageCell*>& unbs)
{
  // Check neighbors vector and return any that are not planned
  unbs.clear();
  for(int i = 0; i<neighbors.size(); i++)
  {
    if(!neighbors[i]->isPlanned())
      unbs.push_back(neighbors[i]);
  }
}

CellValue CoverageCell::getValue()
{
  return value;
}

bool CoverageCell::setValue(CellValue val)
{
  value = val;
  return true;
}

int CoverageCell::getPosition()
{
  return position;
}

void CoverageCell::setPlanned()
{
  cellPlanned = true;
}

void CoverageCell::visit()
{
  cellAttempted = true;
  cellVisited = true;
}

void CoverageCell::setAttempted()
{
  cellAttempted = true;
}

bool CoverageCell::isPlanned()
{
  return cellPlanned;
}

bool CoverageCell::isPlanned(double percent)
{
  return cellPlanned;
}

bool CoverageCell::isVisited()
{
  return cellVisited;
}

bool CoverageCell::isVisited(double percent)
{
  return cellVisited;
}

bool CoverageCell::isAttempted()
{
  return cellAttempted;
}

bool CoverageCell::isAttempted(double percent)
{
  return cellAttempted;
}

void CoverageCell::setReachable(bool ir)
{
  cellReachable = ir;
}

bool CoverageCell::isReachable()
{
  return cellReachable;
}

bool CoverageCell::contains(geometry_msgs::Pose pose)
{
  // if the given pose is within the cell boundaries, return true
  return bounds.contains(pose);
}

geometry_msgs::Pose CoverageCell::getPose()
{
  // Get the center point of this cell in pose form.
  return bounds.getCenter();
}

std::vector<geometry_msgs::Pose> CoverageCell::getCorners()
{
  std::vector<geometry_msgs::Pose> corners;
  bounds.getCorners(corners);
  return corners;
}

void CoverageCell::setCorners(std::vector<geometry_msgs::Pose> allCorners)
{
  bounds.setBounds(allCorners);
}

double CoverageCell::getWidth()
{
  return bounds.getWidth();
}

} // namespace full_coverage

