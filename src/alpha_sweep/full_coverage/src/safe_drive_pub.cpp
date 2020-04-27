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

#include "safe_drive_pub.h"
#include <math.h>
#include "costmap_2d/footprint.h"
#include "geometry_msgs/Point.h"

SafeDrivePub::SafeDrivePub() :
  gotMap(false),
  n("~"),
  spinner(3),
  xHigh(0.0),
  xLow(0.0),
  yLow(0.0),
  yHigh(0.0),
  laserSafe(true),
  driveSafe(true),
  turnSafe(true)
{
  // Load params
  std::string laserTopic, mapTopic;

  if(!n.getParam("safe_distance", safeDistance))
  {
    ROS_ERROR("SafeDrivePub: Could not read safe distance param. setting to default 0.3");
    safeDistance = 0.3;
  }
  
  if(!n.getParam("laser_topic", laserTopic))
  {
    ROS_ERROR("SafeDrivePub: Could not read laser topic param. setting to default /scan");
    laserTopic = "/scan";
  }
  
  if(!n.getParam("costmap_topic", mapTopic))
  {
    ROS_ERROR("SafeDrivePub: Could not read costmap topic param. setting to default /move_base/local_costmap/costmap");
    mapTopic = "/move_base/local_costmap/costmap";
  }
  
  if(!n.getParam("laser_position", laserToFront))
  {
    ROS_ERROR("SafeDrivePub: Could not read laser position param. setting to default 0.0");
    laserToFront = 0.0;
  }
  getFootprint();

  // initialize ROS communications
  safeDrivePub = n.advertise<std_msgs::Bool>("safe_drive", 1);
  safeTurnPub = n.advertise<std_msgs::Bool>("safe_turn", 1);
  safeLaserPub = n.advertise<std_msgs::Bool>("safe_laser", 1);
  laserSub = n.subscribe(laserTopic, 1, &SafeDrivePub::laserCallback, this);
  mapSub = n.subscribe(mapTopic, 1, &SafeDrivePub::mapCallback, this);
  mapUpdateSub = n.subscribe(mapTopic + "_updates", 1, &SafeDrivePub::mapUpdateCallback, this);

  // start ros spinner
  spinner.start();
}

SafeDrivePub::~SafeDrivePub()
{
  // do nothing
}

void SafeDrivePub::getFootprint()
{
  std::vector<geometry_msgs::Point> footprintPoints;
  XmlRpc::XmlRpcValue footprintXMLRPC;
  std::string paramName = "/move_base/local_costmap/footprint";
  n.getParam(paramName, footprintXMLRPC);
  if (footprintXMLRPC.getType() == XmlRpc::XmlRpcValue::TypeString)
  {
    costmap_2d::makeFootprintFromString(std::string(footprintXMLRPC), footprintPoints);
  }
  else if (footprintXMLRPC.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    footprintPoints = costmap_2d::makeFootprintFromXMLRPC(footprintXMLRPC, paramName);
  }

  for(int i = 0; i < footprintPoints.size(); i++)
  {
    if(footprintPoints[i].x < xLow)
      xLow = footprintPoints[i].x;
    if(footprintPoints[i].x > xHigh)
      xHigh = footprintPoints[i].x;
    if(footprintPoints[i].y < yLow)
      yLow = footprintPoints[i].y;
    if(footprintPoints[i].y > yHigh)
      yHigh = footprintPoints[i].y;
  }

  if(footprintPoints.size() < 4)
  {
    ROS_ERROR("SafeDrivePub: footprint not read correctly. safe drive will not work.");
  }
  else
  {
    ROS_INFO_STREAM("SafeDrivePub: using footprint: xHigh: " << xHigh << ", xLow: " << xLow << ", yLow: " << yLow << ", yHigh: " << yHigh);
  }
}

void SafeDrivePub::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  checkScan(msg);
}

void SafeDrivePub::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  map = *msg;
  checkMap();
  gotMap = true;
}

void SafeDrivePub::mapUpdateCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg)
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

  checkMap();
}

void SafeDrivePub::checkMap()
{
  bool needTransform = map.header.frame_id.compare("base_link") != 0;
  int dViolations = 0;
  int tViolations = 0;
  for(int i = 0; i < map.data.size(); i++)
  {
    if(map.data[i] == 100) // 100 corresponds to a lethal obstacle
    {
      tf::Stamped<tf::Point> point;
      int r = floor(i/map.info.width);
      int c = i % map.info.width;
      point.setX(map.info.origin.position.x + c*map.info.resolution); // set the x
      point.setY(map.info.origin.position.y + r*map.info.resolution); // set the y
      if(needTransform)
      {
        point.frame_id_ = map.header.frame_id;
        point.stamp_ = ros::Time(0);
        // do some transform
        try
        {
          if(listener.waitForTransform("base_link", point.frame_id_, ros::Time(0), ros::Duration(1.0)))
          {
            listener.transformPoint("base_link", point, point);
          }
          else
          {
            ROS_ERROR_THROTTLE(1.0, "SafeDrivePub:mapCallback: Timeout waiting for transform to base link.");
          }
        }
        catch(...)
        {
          ROS_ERROR_THROTTLE(1.0, "SafeDrivePub:mapCallback: Exception caught in transform to base link.");
        }
      }

      // check if the point is in the safety zone
      double x = point.getX();
      double y = point.getY();
      if(x > xHigh && x < xHigh + safeDistance && y < yHigh && y > yLow)
        dViolations++;

      if(x > 0 && pow((x*x) + (y*y), 0.5) < pow((xHigh*xHigh) + (yLow*yHigh), 0.5))
        tViolations++;
    }
  }

  if(dViolations > 0)
    driveSafe = false;
  else
    driveSafe = true;

  if(tViolations > 0)
    turnSafe = false;
  else
    turnSafe = true;
}

void SafeDrivePub::checkScan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  double sd = safeDistance; // safe distance to an obstacle (m)
  sd+=laserToFront;
  double rw = (yHigh - yLow) * 1.3; // robot width * safety factor
  double thetaMax = atan(rw/(2*sd)); // max laser theta that gives us obstacles in the front area of the safety zone (radians)
  int minCornerIndex = (-thetaMax - msg->angle_min)/msg->angle_increment;
  int maxCornerIndex = (thetaMax - msg->angle_min)/msg->angle_increment;
  int minIndex = (-1.57 - msg->angle_min)/msg->angle_increment;
  int maxIndex = (1.57 - msg->angle_min)/msg->angle_increment;
  if(minIndex < 0 || minIndex>=msg->ranges.size())
  {
    ROS_ERROR("Safe check: min index out of range.");
    minIndex = 0;
  }
  if(maxIndex < 0 || maxIndex>=msg->ranges.size())
  {
    ROS_ERROR("Safe check: max index out of range.");
    maxIndex = msg->ranges.size()-1;
  }
  if(minCornerIndex < 0 || minCornerIndex>=msg->ranges.size())
  {
    ROS_ERROR("Safe check: min corner index out of range.");
    minCornerIndex = 0;
  }
  if(maxCornerIndex < 0 || maxCornerIndex>=msg->ranges.size())
  {
    ROS_ERROR("Safe check: max corner index out of range.");
    maxCornerIndex = msg->ranges.size()-1;
  }
  //ROS_INFO_STREAM_THROTTLE(1.0,"minI: " << minIndex << " maxI: " << maxIndex << " thetaMax: " << thetaMax);

  int violationCount = 0;
  // Cycle through min to max index seeing if obstacles are in safety zone
  for(int i = minIndex; i< minCornerIndex; i++)
  {
    double thetaI = msg->angle_min + i*msg->angle_increment;
    double sdi = -1.0;
    if(fabs(sin(thetaI)) > 0.001)
    {
      sdi = fabs((rw/2.0)/sin(thetaI));
    }
    //ROS_INFO_STREAM_THROTTLE(1.0, "sdi: " << sdi << " thetaI: " << thetaI);
    if(msg->ranges[i] > 0.0 && msg->ranges[i] < sdi)
      violationCount++;
  }
  for(int i = minCornerIndex; i< maxCornerIndex; i++)
  {
    double thetaI = msg->angle_min + i*msg->angle_increment;
    double sdi = -1.0;
    if(fabs(cos(thetaI)) > 0.001)
    {
      sdi = fabs(sd/cos(thetaI));
    }
    //ROS_INFO_STREAM_THROTTLE(1.0, "sdi: " << sdi << " thetaI: " << thetaI);
    if(msg->ranges[i] > 0.0 && msg->ranges[i] < sdi)
      violationCount++;
  }
  for(int i = maxCornerIndex; i< maxIndex; i++)
  {
    double thetaI = msg->angle_min + i*msg->angle_increment;
    double sdi = -1.0;
    if(fabs(sin(thetaI))> 0.001)
    {
      sdi = fabs((rw/2.0)/sin(thetaI));
    }
    //ROS_INFO_STREAM_THROTTLE(1.0, "sdi: " << sdi << " thetaI: " << thetaI);
    if(msg->ranges[i] > 0.0 && msg->ranges[i] < sdi)
      violationCount++;
  }

  if(violationCount >= 3)
    laserSafe = false;
  else
    laserSafe = true;
}

void SafeDrivePub::publishSafeMsg()
{
  std_msgs::Bool safe_msg;
  if(!driveSafe)
    safe_msg.data = false;
  else
    safe_msg.data = true;

  safeDrivePub.publish(safe_msg);

  if(!turnSafe)
    safe_msg.data = false;
  else
    safe_msg.data = true;

  safeTurnPub.publish(safe_msg);

  if(!laserSafe)
    safe_msg.data = false;
  else
    safe_msg.data = true;
  safeLaserPub.publish(safe_msg);
}

int main(int argc, char **argv)
{
  // Initialize node
  ros::init(argc, argv, "safe_drive_pub");

    // Wall follower object
  SafeDrivePub safeDriver;

  ros::Rate loopRate(10.0);
  while(ros::ok())
  {
    safeDriver.publishSafeMsg();
    loopRate.sleep();
  }

  return 0;
}
