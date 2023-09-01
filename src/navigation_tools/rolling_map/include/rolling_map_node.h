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
#ifndef _ROLLING_MAP_NODE_H_
#define _ROLLING_MAP_NODE_H_

#include "ros/ros.h"
#include "rolling_map.h"
#include "tf/transform_listener.h"
#include "pcl_ros/point_cloud.h"
#include "std_srvs/Empty.h"
#include "nav_msgs/OccupancyGrid.h"
#include "rolling_map/Box.h"
#include "sensor_msgs/PointCloud2.h"
#include "xmlrpcpp/XmlRpc.h"

#ifdef TIMEIT
#include "cpp_timer/Timer.h"
#endif

namespace rolling_map
{
struct MapParams
{
  XmlRpc::XmlRpcValue pc_topics;
  std::string map_topic;
  std::string marker_topic;
  std::string reset_topic;
  std::string world_frame;
  std::string robot_frame;
  int width;
  int height;
  float resolution;
  float z_minimum;
  float run_frequency;
  float translate_distance;
  int ignore_top_rows;
  float sensing_radius;
  float occupancy_threshold_val = 0.0f;
  float occupancy_maximum_val = 1.0f;
  float hit_miss_ratio = 1.0f;
};

class RollingMapNode 
{
private:
  ros::NodeHandle n;
  ros::AsyncSpinner spinner;
  tf::TransformListener listener;
  sensor_msgs::PointCloud2 output_cloud_;
  bool init;

  // Parameters
  MapParams param;

  // Listen for point clouds
  bool hasData;
  std::vector<ros::Subscriber> pc_subs_;
  void pcCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg, const std::string& sensor_frame_id);

  // Publish visual data and map
  ros::Publisher markerPub;
  ros::Publisher mapPub;
  ros::Publisher readyPub;
  ros::Publisher pointcloudPub;
  ros::Publisher outlinePub;
  void publishMessages();
  bool isOccupied(int r, int c, const nav_msgs::OccupancyGrid &g);

  // Reset service
  ros::ServiceServer resetService;
  bool resetCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  // Clear Footprint service
  ros::ServiceServer clearBoxService;
  bool clearBoxCallback(rolling_map::Box::Request &req, rolling_map::Box::Response &res);

  // Map construct
  RollingMap *map;
 
  // Get sensor transform
  bool getTransform(tf::StampedTransform &transform, bool init = false);
  void createAdjustmentVector(const tf::StampedTransform &sensorTransform, std::vector<pcl::PointXYZ> &points);

  // check if map needs to be translated
  tf::StampedTransform robotTransform;
  void checkTranslation();

public:
  RollingMapNode();
  ~RollingMapNode();
  bool isInit();
  void run();

  #ifdef TIMEIT
  std::unique_ptr<cpp_timer::Timer> main_timer;
  std::unique_ptr<cpp_timer::Timer> callback_timer;
  #else
  std::unique_ptr<char> decoy1;
  std::unique_ptr<char> decoy2;
  #endif


};

} // namespace rolling_map

#endif // _ROLLING_MAP_NODE_H_
