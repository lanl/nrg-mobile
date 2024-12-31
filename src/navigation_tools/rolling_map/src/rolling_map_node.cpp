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

#include <csignal>
#include <functional>
#include <execution>

#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

#include "rolling_map_node.h"

#ifdef TIMEIT
#define M_TIC(x)   main_timer->tic(x)
#define M_TOC(x)   main_timer->toc(x)
#define CB_TIC(x)  map->timer->tic(x)
#define CB_TOC(x)  map->timer->toc(x)
#else
#define M_TIC(x)
#define M_TOC(x)
#define CB_TIC(x)
#define CB_TOC(x)
#endif

static std::unique_ptr<rolling_map::RollingMapNode> node;

std::function<void(int)> sigintHandler;
void handle(int signal){
  if(sigintHandler) sigintHandler(signal);
  rclcpp::shutdown();
}

namespace rolling_map
{

RollingMapNode::RollingMapNode() :
  Node("rolling_map_node"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  init(false),
  hasData(false)
{
  #ifdef TIMEIT
  main_timer = std::unique_ptr<cpp_timer::Timer> (new cpp_timer::Timer());

  callback_timer = std::unique_ptr<cpp_timer::Timer> (new cpp_timer::Timer());
  #endif

  ParamListener param_listener(this->get_node_parameters_interface());
  params_ = param_listener.get_params();

  // Find current location of the sensor
  robotTransform.header.frame_id = params_.robot_frame;
  robotTransform.child_frame_id = params_.world_frame;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  if(!getTransform(robotTransform, true))
  {
    RCLCPP_ERROR(get_logger(), "RollingMapNode: Could not look up initial robot transform. cannot initialize map.");
    return;
  }

  ProbabilityModel model{
    .threshold = static_cast<float>(params_.occupancy_threshold),
    .hit_val   = static_cast<float>(params_.hit_value),
    .miss_val  = static_cast<float>(params_.miss_value)
  };

  // Construct map
  map = std::make_shared<RollingMap>(params_.width, params_.height, params_.resolution, robotTransform.transform.translation.x, robotTransform.transform.translation.y, params_.z_minimum, model);

  // Set up ROS communications
  for (const std::string& topic : params_.pointcloud_topics){
    auto new_sub = create_subscription<sensor_msgs::msg::PointCloud2>(topic, rclcpp::SensorDataQoS{}, 
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){
        auto cloud = pcl::PointCloud<pcl::PointXYZ>().makeShared();
        pcl::fromROSMsg(*msg, *cloud);
        this->pcCallback(cloud);
      }
    );
    
    pc_subs_.push_back(new_sub);
  }

  rclcpp::QoS latching_qos = rclcpp::QoS(1).transient_local();
  markerPub = create_publisher<visualization_msgs::msg::Marker>("rolling_map/occupied_cells", latching_qos);
  mapPub = create_publisher<nav_msgs::msg::OccupancyGrid>("rolling_map/projected_map", latching_qos);
  readyPub = create_publisher<std_msgs::msg::Bool>("rolling_map/ready", latching_qos);
  pointcloudPub = create_publisher<sensor_msgs::msg::PointCloud2>("local_pointcloud", latching_qos);
  outlinePub = create_publisher<geometry_msgs::msg::PolygonStamped>("outline", latching_qos);
  resetService = create_service<std_srvs::srv::Empty>("rolling_map/reset", std::bind(&RollingMapNode::resetCallback, this, std::placeholders::_1, std::placeholders::_2));
  clearBoxService = create_service<rolling_map::srv::Box>("rolling_map/clear_box", std::bind(&RollingMapNode::clearBoxCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Set up the output pointcloud
  output_cloud_.header.frame_id = params_.world_frame;
  output_cloud_.height = 1;
  output_cloud_.fields.resize(4);

  constexpr std::array<const char*, 4> field_names = {"x", "y", "z", "one"};
  for (int i = 0; i < 4; i++){
    output_cloud_.fields[i].name   = field_names[i];
    output_cloud_.fields[i].offset = i*4;
    output_cloud_.fields[i].count  = 1;
    output_cloud_.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
  }

  output_cloud_.is_bigendian = false;
  output_cloud_.point_step = 16;
  output_cloud_.is_dense = true;
 
  run_timer_ = create_wall_timer(std::chrono::duration<float>(1/params_.run_frequency), std::bind(&RollingMapNode::run, this));

  init = true;
  RCLCPP_INFO_STREAM(get_logger(), "RollingMap initialized. Initial robot position: (" << robotTransform.transform.translation.x << ", " << robotTransform.transform.translation.y << ")");
  return;
}

bool RollingMapNode::isInit()
{
  return init;
}

bool RollingMapNode::getTransform(geometry_msgs::msg::TransformStamped &transform, bool init)
{
  float duration = 0.5;
  if(init)
    duration = 5.0;
  if(transform.header.frame_id != transform.child_frame_id)
  {
    transform.header.stamp = rclcpp::Time(0);
    try
    {
      if(tf_buffer_.canTransform(transform.child_frame_id, transform.header.frame_id, rclcpp::Time(0), rclcpp::Duration::from_seconds(duration)))
      {
        transform = tf_buffer_.lookupTransform(transform.child_frame_id, transform.header.frame_id, rclcpp::Time(0));
        return true;
      }
      else
      {
        RCLCPP_ERROR_STREAM(get_logger(), "RollingMapNode: getTransform timed out. child_frame: " << transform.child_frame_id << " frame: " << transform.header.frame_id);
        return false;
      }
    }
    catch(...)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "RollingMapNode: exception in getTransform. child_frame: " << transform.child_frame_id << " frame: " << transform.header.frame_id);

      return false;
    }
  }
  else
  {
    // Set identity transform, frames are not different
    transform.transform = geometry_msgs::msg::Transform();
    return true;
  }
}

void RollingMapNode::pcCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
{
  CB_TIC("pcCallback");

  // Remove NaN values, if any.
  CB_TIC("RemoveNaN");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*msg, *cloud, indices);
  CB_TOC("RemoveNaN");

  // Transform point cloud to world frame
  CB_TIC("TransformPointcloud");
  geometry_msgs::msg::TransformStamped dataTransform;
  dataTransform.header.frame_id = cloud->header.frame_id;
  dataTransform.child_frame_id = params_.world_frame;
  if(!getTransform(dataTransform))
  {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "RollingMapNode: Could not insert point cloud because we could not look up data transform from %s to %s", cloud->header.frame_id.c_str(), params_.world_frame.c_str());
    CB_TOC("TransformPointcloud");
    CB_TOC("pcCallback");
    return;
  }
  
  Eigen::Isometry3d data_transform_eigen = tf2::transformToEigen(dataTransform);
  pcl::transformPointCloud(*cloud, *cloud, data_transform_eigen.matrix());
  CB_TOC("TransformPointcloud");

  // Find the sensor origin in the world frame
  geometry_msgs::msg::TransformStamped sensorTransform;
  sensorTransform.header.frame_id = cloud->header.frame_id;
  sensorTransform.child_frame_id = params_.world_frame;
  if (!getTransform(sensorTransform)){
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "RollingMapNode: Could not insert point cloud because we could not look up sensor transform from %s to %s", cloud->header.frame_id.c_str(), params_.world_frame.c_str());
    CB_TOC("pcCallback");
    return;
  }

  // Pull out vector of points and insert cloud
  CB_TIC("insertCloud");
  std::vector<pcl::PointXYZ> points(cloud->begin(),cloud->end());
  pcl::PointXYZ origin(sensorTransform.transform.translation.x, sensorTransform.transform.translation.y, sensorTransform.transform.translation.z);
  map->insertCloud(points,origin);
  hasData = true;
  CB_TOC("insertCloud");

  CB_TOC("pcCallback");
}

void RollingMapNode::resetCallback(std_srvs::srv::Empty::Request::ConstSharedPtr req, std_srvs::srv::Empty::Response::SharedPtr res)
{
  map->clearAll();
}

void RollingMapNode::clearBoxCallback(rolling_map::srv::Box::Request::ConstSharedPtr req, rolling_map::srv::Box::Response::SharedPtr res)
{
  RCLCPP_INFO_STREAM(get_logger(), "RollingMapNode: Executing clearBoxCallback");

  if(req->p1.header.frame_id != req->p2.header.frame_id)
  {
    RCLCPP_ERROR(get_logger(), "RollingMapNode: Cannot clear box, point frames are different");
    return;
  }
 
  // Add two points to make bounds of a rectangle 
  // (note that we are assuming the z axis it perpendicular to the ground.
  //  this is usually the convention)
  geometry_msgs::msg::PointStamped p3 = req->p1;
  p3.point.x = req->p2.point.x;
  geometry_msgs::msg::PointStamped p4 = req->p1;
  p4.point.y = req->p2.point.y;

  // Transform points to map frame
  if(!tf_buffer_.canTransform(req->p1.header.frame_id, params_.world_frame, req->p1.header.stamp, rclcpp::Duration::from_seconds(0.5)))
  {
    RCLCPP_ERROR(get_logger(), "RollingMapNode: Cannot clear box, failed to get transform from point frame to map frame");
    return;
  }
  geometry_msgs::msg::PointStamped point1 = tf_buffer_.transform(req->p1,point1, params_.world_frame);
  geometry_msgs::msg::PointStamped point2 = tf_buffer_.transform(req->p2,point2, params_.world_frame);
  geometry_msgs::msg::PointStamped point3 = tf_buffer_.transform(p3,point3, params_.world_frame);
  geometry_msgs::msg::PointStamped point4 = tf_buffer_.transform(p4,point4, params_.world_frame);

  // Put points in array that is ordered consecutively
  std::vector<std::array<float, 2>> polygon;
  std::array<float, 2> point;
  point[0] = point1.point.x;
  point[1] = point1.point.y;
  polygon.push_back(point);
  point[0] = point3.point.x;
  point[1] = point3.point.y;
  polygon.push_back(point);
  point[0] = point2.point.x;
  point[1] = point2.point.y;
  polygon.push_back(point);
  point[0] = point4.point.x;
  point[1] = point4.point.y;
  polygon.push_back(point);
    
  // Clear box bounded by the two points
  if(!map->clearPositionBox(polygon, req->p1.point.z, req->p2.point.z))
  {
    RCLCPP_ERROR(get_logger(), "RollingMapNode: Rolling map failed to clear position box.");
    return;
  }

  RCLCPP_INFO_STREAM(get_logger(), "RollingMapNode: Cleared map box from (" << req->p1.point.x << ", " << req->p1.point.y << ", " << req->p1.point.z << ") to (" << req->p2.point.x << ", " << req->p2.point.y << ", " << req->p2.point.z << ") in frame: " << req->p1.header.frame_id);  
}

void RollingMapNode::checkTranslation()
{
  geometry_msgs::msg::TransformStamped tempTransform;
  tempTransform.header.frame_id = params_.robot_frame;
  tempTransform.child_frame_id  = params_.world_frame;
  getTransform(tempTransform);
  float xDiff = tempTransform.transform.translation.x - robotTransform.transform.translation.x;
  float yDiff = tempTransform.transform.translation.y - robotTransform.transform.translation.y;
  float dist = pow(pow(xDiff,2) + pow(yDiff,2), 0.5);
  if(dist > params_.translate_distance)
  {
    robotTransform = tempTransform;
    M_TIC("updatePosition");
    map->updatePosition(robotTransform.transform.translation.x, robotTransform.transform.translation.y);
    M_TOC("updatePosition");
  }
}

void RollingMapNode::publishMessages()
{
  M_TIC("publishMessages");

  // Get a copy of the current map
  M_TIC("getMap");
  std::vector<Coord> points = map->getMap();
  M_TOC("getMap");

  const float minXP = map->getMinXP();
  const float minZP = map->getMinZP();
  const float minYP = map->getMinYP();

  M_TIC("publishMap");
  if(mapPub->get_subscription_count() > 0)
  {
    // Set up grid info
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = now();
    grid.header.frame_id = params_.world_frame;
    grid.info.resolution = map->getResolution();
    grid.info.width = map->getWidth();
    grid.info.height = map->getWidth();
    grid.info.origin.position.x = minXP;
    grid.info.origin.position.y = minYP;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.x = 0.0;
    grid.info.origin.orientation.y = 0.0;
    grid.info.origin.orientation.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    grid.data.resize(map->getWidth()*map->getWidth(),0);

    // Sum z columns to build 2d map
    for(const Coord& c : points)
    {
      if(c.z <= map->getMaxZI() - params_.ignore_top_rows)
      {
        int index = c.y*grid.info.width + c.x; 
        if(index >= 0 && index <= grid.data.size())
          grid.data[index] = 100;
        else
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "RollingMapNode: Map publish calculated invalid index");
      }
    }

    // Cycle through map and get rid of lonely cells
    for(int i = 0; i < grid.data.size(); i++)
    {
      int c = i%grid.info.width;
      int r = (i-c)/grid.info.width;
      if(!isOccupied(r,c-1,grid) && !isOccupied(r,c+1,grid) &&
         !isOccupied(r-1,c,grid) && !isOccupied(r+1,c,grid))
      {
        grid.data[i] = 0;
      } 
    }

    mapPub->publish(grid);
  }
  M_TOC("publishMap");

  // Convert integer point indices to float coordinates
  M_TIC("Index2Float");
  const float res = map->getResolution();
  std::vector<pcl::PointXYZ> true_points(points.size());
  std::transform(std::begin(points), std::end(points), true_points.begin(), [&](const Coord& c) -> pcl::PointXYZ {
    pcl::PointXYZ p;
    p.x = minXP + c.x*res;
    p.y = minYP + c.y*res;
    p.z = minZP + c.z*res;
    return p;
  });
  M_TOC("Index2Float");

  // M_TIC("publishMarkers");
  // if(markerPub.getNumSubscribers() > 0)
  // {
  //   // Add points to marker array
  //   visualization_msgs::msg::Marker occupied;
  //   occupied.header.frame_id = params_.world_frame;
  //   occupied.header.stamp = ros::Time::now();
  //   occupied.ns = "map";
  //   occupied.id = 1;
  //   occupied.type = visualization_msgs::msg::Marker::CUBE_LIST;
  //   occupied.action = visualization_msgs::msg::Marker::ADD;
  //   occupied.pose.orientation.w = 1.0;
  //   occupied.scale.x = map->getResolution();
  //   occupied.scale.y = map->getResolution();
  //   occupied.scale.z = map->getResolution();
  //   occupied.colors.resize(points.size());

  //   for(int i = 0; i < points.size(); i++)
  //   {
  //     geometry_msgs::msg::Point center;
  //     center.x = points[i].x;
  //     center.y = points[i].y;
  //     center.z = points[i].z;
  //     occupied.points.push_back(center);
  //     float heightPercent = points[i].z/map->getHeight()/res;
  //     std_msgs::msg::ColorRGBA color;
  //     color.r = 0;
  //     color.g = heightPercent;
  //     color.b = 1-heightPercent;
  //     color.a = 1;
  //     occupied.colors[i] = color;
  //   }
  //   markerPub.publish(occupied);
  // }
  // M_TOC("publishMarkers");


  // Copy the data to the pointcloud message
  M_TIC("publishPointcloud");
  output_cloud_.header.stamp = now();
  output_cloud_.width = points.size();
  output_cloud_.data.clear();
  output_cloud_.data.resize(output_cloud_.point_step * output_cloud_.width);
  memcpy(output_cloud_.data.data(), true_points.data(), output_cloud_.data.size());
  pointcloudPub->publish(output_cloud_);
  M_TOC("publishPointcloud");
  
  // Ready pub
  std_msgs::msg::Bool msg;
  msg.data = hasData;
  readyPub->publish(msg);

  // Outline pub
  geometry_msgs::msg::PolygonStamped outline;
  outline.header.stamp = now();
  outline.header.frame_id = params_.world_frame;
  outline.polygon.points.reserve(8);
  outline.polygon.points.resize(4);
  outline.polygon.points[0].x = minXP;
  outline.polygon.points[0].y = minYP;
  outline.polygon.points[1].x = minXP + map->getWidth()*map->getResolution();
  outline.polygon.points[1].y = minYP;
  outline.polygon.points[2].x = minXP + map->getWidth()*map->getResolution();
  outline.polygon.points[2].y = minYP + map->getWidth()*map->getResolution();
  outline.polygon.points[3].x = minXP;
  outline.polygon.points[3].y = minYP + map->getWidth()*map->getResolution();
  for (int i = 0; i < 4; i++){
    outline.polygon.points[i].z = minZP;
    outline.polygon.points.push_back(outline.polygon.points[i]);
    outline.polygon.points.back().z += map->getHeight()*map->getResolution();
  }
  outlinePub->publish(outline);

  M_TOC("publishMessages");
}

bool RollingMapNode::isOccupied(int r, int c, const nav_msgs::msg::OccupancyGrid &g)
{
  if(r >= 0 && c >= 0 && r < g.info.height && c < g.info.width)
  {
    int i = r*g.info.width + c;
    if(i >= 0 && i < g.data.size())
    {
      if(g.data[i] > 0)
        return true;
    }
  }  
  return false;
}

void RollingMapNode::run()
{
  publishMessages();
  checkTranslation();
}

} // namespace rolling_map

int main(int argc, char** argv)
{
  rclcpp::InitOptions opts;
  opts.shutdown_on_signal = false;
  rclcpp::init(argc, argv, opts);
  
  auto node = std::make_shared<rolling_map::RollingMapNode>();

  sigintHandler = [node](int signal){
    #ifdef TIMEIT
    node->main_timer->summary(cpp_timer::Timer::BY_AVERAGE);
    node->callback_timer->summary(cpp_timer::Timer::BY_AVERAGE);
    #endif
  };
  std::signal(SIGINT, handle);

  rclcpp::spin(node);

  return 0;
}
