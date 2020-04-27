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
* Description: Holds cell boundary data.
*********************************************************************/

#include "cell_bounds.h"

namespace full_coverage
{

CellBounds::CellBounds() :
  shortest(0),
  longest(0)
{
  corners.clear();
}

CellBounds::~CellBounds()
{
  // Do nothing
}

void CellBounds::setBounds(std::vector<geometry_msgs::Pose> allCorners)
{
  if(allCorners.size() >=4)
  {
    corners.clear();
    shortest = 0;
    longest = 0;
    if(allCorners.size()>4)
    {
      // If we are passed more than 4 corners, then we need to find the 4 that are at the extremities
      tf::Point tl(allCorners[0].position.x,allCorners[0].position.y,0.0);
      tf::Point tr = tl;
      tf::Point bl = tl;
      tf::Point br = tl;
      for(int i = 1; i<allCorners.size(); i++)
      {
        // If point is on same vertical line (rectange not tilted), we want top-most
        if(fabs(allCorners[i].position.x - tl.getX()) < 0.00001)
        {
          if(allCorners[i].position.y > tl.getY())
          {
            tl.setX(allCorners[i].position.x);
            tl.setY(allCorners[i].position.y);
          }
        }
        // Otherwise (rectangle is tilted) we just want the farthest left
        else if(allCorners[i].position.x < tl.getX())
        {
          tl.setX(allCorners[i].position.x);
          tl.setY(allCorners[i].position.y);
        }

        // If point is on same horizontal line (rectange not tilted), we want right-most
        if(fabs(allCorners[i].position.y - tr.getY()) < 0.00001)
        {
          if(allCorners[i].position.x > tr.getX())
          {
            tr.setX(allCorners[i].position.x);
            tr.setY(allCorners[i].position.y);
          }
        }
        // Otherwise (rectangle is tilted) we just want the highest
        else if(allCorners[i].position.y > tr.getY())
        {
          tr.setX(allCorners[i].position.x);
          tr.setY(allCorners[i].position.y);
        }

        // If point is on same vertical line (rectange not tilted), we want bottom-most
        if(fabs(allCorners[i].position.x - br.getX()) < 0.00001)
        {
          if(allCorners[i].position.y < br.getY())
          {
            br.setX(allCorners[i].position.x);
            br.setY(allCorners[i].position.y);
          }
        }
        // Otherwise (rectangle is tilted) we just want the farthest right
        else if(allCorners[i].position.x > br.getX())
        {
          br.setX(allCorners[i].position.x);
          br.setY(allCorners[i].position.y);
        }

        // If point is on same horizontal line (rectange not tilted), we want left-most
        if(fabs(allCorners[i].position.y - bl.getY()) < 0.00001)
        {
          if(allCorners[i].position.x < bl.getX())
          {
            bl.setX(allCorners[i].position.x);
            bl.setY(allCorners[i].position.y);
          }
        }
        // Otherwise (rectangle is tilted) we just want the lowest
        else if(allCorners[i].position.y < bl.getY())
        {
          bl.setX(allCorners[i].position.x);
          bl.setY(allCorners[i].position.y);
        }
      }

      corners.push_back(tl);
      corners.push_back(tr);
      corners.push_back(bl);
      corners.push_back(br);
    }
    else
    {
      for(int i = 0; i<allCorners.size(); i++)
      {
        corners.push_back(tf::Point(allCorners[i].position.x, allCorners[i].position.y, 0.0));
      }
    }

    for(int i = 1; i<corners.size(); i++)
    {
      if(corners[i].length() < corners[shortest].length())
      {
        shortest = i;
      }
      if(corners[i].length() > corners[longest].length())
      {
        longest = i;
      }
    }
  }
  else
    ROS_ERROR("CellBounds: Cannot set bounds. setBounds was passed incorrect number of corners.");
}

void CellBounds::getCorners(std::vector<geometry_msgs::Pose> &c)
{
  if(corners.size() != 4)
  {
    ROS_ERROR("Cell Bounds: getCorners() called on unitialized cell bounds.");
    return;
  }

  c.clear();
  geometry_msgs::Pose tempP;
  tempP.position.z = 0.0;
  tempP.orientation.x = 0.0;
  tempP.orientation.y = 0.0;
  tempP.orientation.z = 0.0;
  tempP.orientation.w = 1.0;
  for(int i = 0; i < 4; i++)
  {
    tempP.position.x = corners[i].getX();
    tempP.position.y = corners[i].getY();
    c.push_back(tempP);
  }
}

double CellBounds::getWidth()
{
  if(corners.size() != 4)
  {
    ROS_ERROR("Cell Bounds: getWidth() called on unitialized cell bounds.");
    return 0.0;
  }

  double l1 = fabs(corners[longest].getX() - corners[shortest].getX());
  double l2 = fabs(corners[longest].getY() - corners[shortest].getY());
  if(l1>l2)
    return l1;
  else
    return l2;
}

geometry_msgs::Pose CellBounds::getCenter()
{
  geometry_msgs::Pose center;
  if(corners.size() != 4)
  {
    center.position.x = 0.0;
    center.position.y = 0.0;
    ROS_ERROR("Cell Bounds: getCenter() called on unitialized cell bounds.");
  }
  else
  {
    center.position.x = (corners[longest].getX() + corners[shortest].getX())/2.0;
    center.position.y = (corners[longest].getY() + corners[shortest].getY())/2.0;
  }
  center.position.z = 0.0;
  center.orientation.x = 0.0;
  center.orientation.y = 0.0;
  center.orientation.z = 0.0;
  center.orientation.w = 1.0;
  return center;
}

bool CellBounds::contains(geometry_msgs::Pose pose)
{
  if(corners.size() != 4)
  {
    ROS_ERROR("Cell Bounds: contains() called on unitialized cell bounds.");
    return false;
  }

  std::vector<int> mid;
  tf::Point point(pose.position.x, pose.position.y, 0.0);
  for(int i = 0; i < corners.size(); i++)
  {
    if(i!=shortest && i!=longest)
      mid.push_back(i);
  }
  tf::Point v1 = corners[mid[0]] - corners[shortest];
  tf::Point v2 = corners[mid[1]] - corners[shortest];
  point -= corners[shortest];

  if(fabs(point.angle(v1))<M_PI/2.0 && fabs(point.angle(v2))<M_PI/2.0 &&
     point.dot(v1)/v1.length() < v1.length() && point.dot(v2)/v2.length() < v2.length())
  {
    return true;
  }
  else
  {
    return false;
  }
}

} // namespace full_coverage

