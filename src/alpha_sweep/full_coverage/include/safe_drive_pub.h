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

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/OccupancyGrid.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include "tf/transform_listener.h"

class SafeDrivePub
{
public:
  /* Wall follower constructor
   * @param nh         ROS Node handle
   * @param scanTopic  ROS topic name for laser scan topic
   * @param bumpTopic  ROS topic name for bumper status
   */
  SafeDrivePub();

  /* Wall follower destrcutor
   */
  ~SafeDrivePub();
  void publishSafeMsg();

private:
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void checkScan(const sensor_msgs::LaserScan::ConstPtr& msg);
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void mapUpdateCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg);
  void checkMap();
  void getFootprint();
  nav_msgs::OccupancyGrid map;
  bool gotMap;

  ros::NodeHandle n;              // ROS nodehandle
  ros::AsyncSpinner spinner;
  tf::TransformListener listener;  // ROS tranform listener
  double xLow, xHigh, yLow, yHigh;

  bool laserSafe;   // true if there are no laser points in safe zone
  bool driveSafe;   // true if there are no lethal obstacles in costmap in forward safe zone
  bool turnSafe;    // true if the costmap shows not obstacles in the turning radius

  ros::Publisher safeDrivePub;   // ROS Publisher for if it is safe to drive forward
  ros::Publisher safeTurnPub;   // ROS Publisher for if it is safe to drive forward
  ros::Publisher safeLaserPub;   // ROS Publisher for if it is safe to drive forward
  ros::Subscriber laserSub; // ROS Subscriber for laser scan.
  ros::Subscriber mapSub; // ROS Subscriber for local costmap.
  ros::Subscriber mapUpdateSub; // ROS Subscriber for local costmap updates.

  double laserToFront;
  double safeDistance;
};
