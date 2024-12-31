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

#include "rolling_map.h"
#include "rolling_map/srv/box.hpp"
#include "rolling_map_parameters.hpp" // Auto-generated parameter header

#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#ifdef TIMEIT
#include "cpp_timer/Timer.h"
#endif

namespace rolling_map
{
class RollingMapNode : public rclcpp::Node
{
private:
  Params params_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  sensor_msgs::msg::PointCloud2 output_cloud_;
  bool init;

  // Listen for point clouds
  bool hasData;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> pc_subs_;
  void pcCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg);

  // Publish visual data and map
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPub;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr readyPub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloudPub;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr outlinePub;
  void publishMessages();
  bool isOccupied(int r, int c, const nav_msgs::msg::OccupancyGrid &g);

  // Reset service
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr resetService;
  void resetCallback(std_srvs::srv::Empty::Request::ConstSharedPtr req, std_srvs::srv::Empty::Response::SharedPtr res);

  // Clear Footprint service
  rclcpp::Service<rolling_map::srv::Box>::SharedPtr clearBoxService;
  void clearBoxCallback(rolling_map::srv::Box::Request::ConstSharedPtr req, rolling_map::srv::Box::Response::SharedPtr res);

  // Map construct
  std::shared_ptr<RollingMap> map;
 
  // Get sensor transform
  bool getTransform(geometry_msgs::msg::TransformStamped &transform, bool init = false);

  // check if map needs to be translated
  geometry_msgs::msg::TransformStamped robotTransform;
  void checkTranslation();

  // Main loop
  rclcpp::TimerBase::SharedPtr run_timer_;
  void run();

public:
  RollingMapNode();
  bool isInit();

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
