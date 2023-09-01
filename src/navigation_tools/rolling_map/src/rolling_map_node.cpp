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

#include "rolling_map_node.h"
#include "visualization_msgs/Marker.h"
#include "pcl_ros/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/filter.h"
#include "std_msgs/Bool.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/PolygonStamped.h"
#include <csignal>
#include <functional>
#include <execution>

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
  ros::shutdown();
}

namespace rolling_map
{

RollingMapNode::RollingMapNode() :
  n("~"),
  spinner(1),
  init(false),
  hasData(false)
{
  #ifdef TIMEIT
  main_timer = std::unique_ptr<cpp_timer::Timer> (new cpp_timer::Timer());

  callback_timer = std::unique_ptr<cpp_timer::Timer> (new cpp_timer::Timer());
  #endif

  // Get params
  if(!n.getParam("map_topic", param.map_topic)                   
  || !n.getParam("marker_topic", param.marker_topic)             
  || !n.getParam("reset_topic", param.reset_topic)               
  || !n.getParam("world_frame", param.world_frame)               
  || !n.getParam("robot_frame", param.robot_frame)               
  || !n.getParam("width", param.width)                           
  || !n.getParam("height", param.height)                         
  || !n.getParam("resolution", param.resolution)                 
  || !n.getParam("z_minimum", param.z_minimum)                   
  || !n.getParam("run_frequency", param.run_frequency)           
  || !n.getParam("translate_distance", param.translate_distance) 
  || !n.getParam("ignore_top_rows", param.ignore_top_rows)       
  || !n.getParam("sensing_radius", param.sensing_radius))
  {
    ROS_ERROR("RollingMapNode: Cannot construct map. Some params could not be read from server.");
    return;
  }

  // Get all of the sensor sources
  bool has_topics = n.getParam("pc_topics", param.pc_topics);
  if (not has_topics || param.pc_topics.getType() != XmlRpc::XmlRpcValue::TypeArray || param.pc_topics.size() == 0){
    ROS_ERROR("RollingMapNode: Cannot construct map. Pointcloud topics not set.");
    return;
  }

  // Find current location of the sensor
  robotTransform.frame_id_ = param.robot_frame;
  robotTransform.child_frame_id_ = param.world_frame;
  ros::Duration(3).sleep();
  if(!getTransform(robotTransform, true))
  {
    ROS_ERROR("RollingMapNode: Could not look up initial robot transform. cannot initialize map.");
    return;
  }

  // Construct map
  map = new RollingMap(param.width, param.height, param.resolution, robotTransform.getOrigin().getX(), robotTransform.getOrigin().getY(), param.z_minimum);

  // Set up ROS communications
  for (int i = 0; i < param.pc_topics.size(); i++){
    std::string topic = static_cast<std::string>(param.pc_topics[i]["topic"]);
    std::string sensor_frame = static_cast<std::string>(param.pc_topics[i]["sensor_frame"]);
    pc_subs_.emplace_back(n.subscribe<pcl::PointCloud<pcl::PointXYZ>>(topic, 1, 
      [this, sensor_frame](const auto& msg){
        this->pcCallback(msg, sensor_frame);
      }
    ));
  }
  markerPub = n.advertise<visualization_msgs::Marker>(param.marker_topic, 1, true);
  mapPub = n.advertise<nav_msgs::OccupancyGrid>(param.map_topic,1,true);
  readyPub = n.advertise<std_msgs::Bool>("ready",1,true);
  pointcloudPub = n.advertise<sensor_msgs::PointCloud2>("local_pointcloud", 1, true);
  outlinePub = n.advertise<geometry_msgs::PolygonStamped>("outline", 1, true);
  resetService = n.advertiseService(param.reset_topic, &RollingMapNode::resetCallback, this);
  clearBoxService = n.advertiseService("clear_box", &RollingMapNode::clearBoxCallback, this);

  // Set up the output pointcloud
  output_cloud_.header.frame_id = param.world_frame;
  output_cloud_.header.seq = 0;
  output_cloud_.height = 1;
  output_cloud_.fields.resize(4);

  constexpr std::array<const char*, 4> field_names = {"x", "y", "z", "one"};
  for (int i = 0; i < 4; i++){
    output_cloud_.fields[i].name   = field_names[i];
    output_cloud_.fields[i].offset = i*4;
    output_cloud_.fields[i].count  = 1;
    output_cloud_.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
  }

  output_cloud_.is_bigendian = false;
  output_cloud_.point_step = 16;
  output_cloud_.is_dense = true;
 
  spinner.start();

  init = true;
  ROS_INFO_STREAM("RollingMap initialized. Initial robot position: (" << robotTransform.getOrigin().getX() << ", " << robotTransform.getOrigin().getY() << ")");
  return;
}

RollingMapNode::~RollingMapNode()
{
  delete map;
  // do nothing
}

bool RollingMapNode::isInit()
{
  return init;
}

bool RollingMapNode::getTransform(tf::StampedTransform &transform, bool init)
{
  float duration = 0.5;
  if(init)
    duration = 5.0;
  if(transform.frame_id_.compare(transform.child_frame_id_) != 0)
  {
    transform.stamp_ = ros::Time(0);
    try
    {
      if(listener.waitForTransform(transform.child_frame_id_, transform.frame_id_, ros::Time(0), ros::Duration(duration)))
      {
        listener.lookupTransform(transform.child_frame_id_, transform.frame_id_, ros::Time(0), transform);
        return true;
      }
      else
      {
        ROS_ERROR_STREAM("RollingMapNode: getTransform timed out. child_frame: " << transform.child_frame_id_ << " frame: " << transform.frame_id_);
        return false;
      }
    }
    catch(...)
    {
      ROS_ERROR_STREAM("RollingMapNode: exception in getTransform. child_frame: " << transform.child_frame_id_ << " frame: " << transform.frame_id_);

      return false;
    }
  }
  else
  {
    // Set identity transform, frames are not different
    transform.setIdentity();
    return true;
  }
}

void RollingMapNode::pcCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg, const std::string& sensor_frame_id)
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
  tf::StampedTransform dataTransform;
  dataTransform.frame_id_ = cloud->header.frame_id;
  dataTransform.child_frame_id_ = param.world_frame;
  if(!getTransform(dataTransform))
  {
    ROS_ERROR_THROTTLE(1.0, "RollingMapNode: Could not insert point cloud because we could not look up data transform from %s to %s", cloud->header.frame_id.c_str(), param.world_frame.c_str());
    CB_TOC("TransformPointcloud");
    CB_TOC("pcCallback");
    return;
  }
  pcl_ros::transformPointCloud(*cloud,*cloud,dataTransform);
  CB_TOC("TransformPointcloud");

  // Find the sensor origin in the world frame
  tf::StampedTransform sensorTransform;
  sensorTransform.frame_id_ = sensor_frame_id == "" ? cloud->header.frame_id : sensor_frame_id;
  sensorTransform.child_frame_id_ = param.world_frame;
  if (!getTransform(sensorTransform)){
    ROS_ERROR_THROTTLE(1.0, "RollingMapNode: Could not insert point cloud because we could not look up sensor transform from %s to %s", sensor_frame_id.c_str(), param.world_frame.c_str());
    CB_TOC("pcCallback");
    return;
  }

  // Pull out vector of points and insert cloud
  CB_TIC("insertCloud");
  std::vector<pcl::PointXYZ> points(cloud->begin(),cloud->end());
  pcl::PointXYZ origin(sensorTransform.getOrigin().x(),sensorTransform.getOrigin().y(),sensorTransform.getOrigin().z());
  map->insertCloud(points,origin);
  hasData = true;
  CB_TOC("insertCloud");

  CB_TOC("pcCallback");
}

void RollingMapNode::createAdjustmentVector(const tf::StampedTransform &sensorTransform, std::vector<pcl::PointXYZ> &points)
{
  points.clear();
  if(param.sensing_radius > 0.000001)
  {
    for(int i = 0; i < 360; i++)
    {
      // Make sensor circle and transform to map frame
      tf::Vector3 v;
      v.setX(param.sensing_radius * cos(i*M_PI/180.0));
      v.setY(param.sensing_radius * sin(i*M_PI/180.0));
      v.setZ(0.0);
      v = sensorTransform(v);

      // Subtract off translation from map to sensor
      pcl::PointXYZ p;
      p.x = v.x() - sensorTransform.getOrigin().x();
      p.y = v.y() - sensorTransform.getOrigin().y();
      p.z = v.z() - sensorTransform.getOrigin().z();

      // Push back to points vector
      points.push_back(p);
    }
  }
}

bool RollingMapNode::resetCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  map->clearAll();
  return true;
}

bool RollingMapNode::clearBoxCallback(rolling_map::Box::Request &req, rolling_map::Box::Response &res)
{
  ROS_INFO_STREAM("RollingMapNode: Executing clearBoxCallback");

  if(req.p1.header.frame_id.compare(req.p2.header.frame_id) != 0)
  {
    ROS_ERROR("RollingMapNode: Cannot clear box, point frames are different");
    return false;
  }
 
  // Add two points to make bounds of a rectangle 
  // (note that we are assuming the z axis it perpendicular to the ground.
  //  this is usually the convention)
  geometry_msgs::PointStamped p3 = req.p1;
  p3.point.x = req.p2.point.x;
  geometry_msgs::PointStamped p4 = req.p1;
  p4.point.y = req.p2.point.y;

  // Transform points to map frame
  tf::Stamped<tf::Point> point1, point2, point3, point4;
  tf::pointStampedMsgToTF(req.p1,point1);
  tf::pointStampedMsgToTF(req.p2,point2);
  tf::pointStampedMsgToTF(p3,point3);
  tf::pointStampedMsgToTF(p4,point4);
  tf::StampedTransform t;
  t.frame_id_ = req.p1.header.frame_id;
  t.child_frame_id_ = param.world_frame;
  t.stamp_ = ros::Time::now();
  if(!getTransform(t))
  {
    ROS_ERROR("RollingMapNode: Cannot clear box, failed to get transform from point frame to map frame");
    return false;
  }
  point1.setData(t*point1);
  point2.setData(t*point2);
  point3.setData(t*point3);
  point4.setData(t*point4);

  // Put points in array that is ordered consecutively
  std::vector<std::vector<float>> polygon;
  std::vector<float> point;
  point.resize(2);
  point[0] = point1.getX();
  point[1] = point1.getY();
  polygon.push_back(point);
  point[0] = point3.getX();
  point[1] = point3.getY();
  polygon.push_back(point);
  point[0] = point2.getX();
  point[1] = point2.getY();
  polygon.push_back(point);
  point[0] = point4.getX();
  point[1] = point4.getY();
  polygon.push_back(point);
    
  // Clear box bounded by the two points
  if(!map->clearPositionBox(polygon, req.p1.point.z, req.p2.point.z))
  {
    ROS_ERROR("RollingMapNode: Rolling map failed to clear position box.");
    return false;
  }

  ROS_INFO_STREAM("RollingMapNode: Cleared map box from (" << req.p1.point.x << ", " << req.p1.point.y << ", " << req.p1.point.z << ") to (" << req.p2.point.x << ", " << req.p2.point.y << ", " << req.p2.point.z << ") in frame: " << req.p1.header.frame_id);  
  return true;
}

void RollingMapNode::checkTranslation()
{
  tf::StampedTransform tempTransform;
  tempTransform.frame_id_ = param.robot_frame;
  tempTransform.child_frame_id_ = param.world_frame;
  getTransform(tempTransform);
  float xDiff = tempTransform.getOrigin().getX() - robotTransform.getOrigin().getX();
  float yDiff = tempTransform.getOrigin().getY() - robotTransform.getOrigin().getY();
  float dist = pow(pow(xDiff,2) + pow(yDiff,2), 0.5);
  if(dist > param.translate_distance)
  {
    robotTransform = tempTransform;
    M_TIC("updatePosition");
    map->updatePosition(robotTransform.getOrigin().getX(), robotTransform.getOrigin().getY());
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
  if(mapPub.getNumSubscribers() > 0)
  {
    // Set up grid info
    nav_msgs::OccupancyGrid grid;
    grid.header.stamp = ros::Time::now();
    grid.header.frame_id = param.world_frame;
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
      if(c.z <= map->getMaxZI() - param.ignore_top_rows)
      {
        int index = c.y*grid.info.width + c.x; 
        if(index >= 0 && index <= grid.data.size())
          grid.data[index] = 100;
        else
          ROS_ERROR_THROTTLE(1.0, "RollingMapNode: Map publish calculated invalid index");
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

    mapPub.publish(grid);
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
  //   visualization_msgs::Marker occupied;
  //   occupied.header.frame_id = param.world_frame;
  //   occupied.header.stamp = ros::Time::now();
  //   occupied.ns = "map";
  //   occupied.id = 1;
  //   occupied.type = visualization_msgs::Marker::CUBE_LIST;
  //   occupied.action = visualization_msgs::Marker::ADD;
  //   occupied.pose.orientation.w = 1.0;
  //   occupied.scale.x = map->getResolution();
  //   occupied.scale.y = map->getResolution();
  //   occupied.scale.z = map->getResolution();
  //   occupied.colors.resize(points.size());

  //   for(int i = 0; i < points.size(); i++)
  //   {
  //     geometry_msgs::Point center;
  //     center.x = points[i].x;
  //     center.y = points[i].y;
  //     center.z = points[i].z;
  //     occupied.points.push_back(center);
  //     float heightPercent = points[i].z/map->getHeight()/res;
  //     std_msgs::ColorRGBA color;
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
  output_cloud_.header.stamp = ros::Time::now();
  output_cloud_.header.seq++;
  output_cloud_.width = points.size();
  output_cloud_.data.clear();
  output_cloud_.data.resize(output_cloud_.point_step * output_cloud_.width);
  memcpy(output_cloud_.data.data(), true_points.data(), output_cloud_.data.size());
  pointcloudPub.publish(output_cloud_);
  M_TOC("publishPointcloud");
  
  // Ready pub
  std_msgs::Bool msg;
  msg.data = hasData;
  readyPub.publish(msg);

  // Outline pub
  geometry_msgs::PolygonStamped outline;
  outline.header.stamp = ros::Time::now();
  outline.header.frame_id = param.world_frame;
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
  outlinePub.publish(outline);

  M_TOC("publishMessages");
}

bool RollingMapNode::isOccupied(int r, int c, const nav_msgs::OccupancyGrid &g)
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
  ros::Rate pubRate(param.run_frequency);
  while(ros::ok())
  {
    publishMessages();
    checkTranslation();
    pubRate.sleep();
  }
}

} // namespace rolling_map

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rolling_map", ros::init_options::NoSigintHandler);

  node = std::make_unique<rolling_map::RollingMapNode>();

  sigintHandler = [](int signal){
    #ifdef TIMEIT
    node->main_timer->summary(cpp_timer::Timer::BY_AVERAGE);
    node->callback_timer->summary(cpp_timer::Timer::BY_AVERAGE);
    node->~RollingMapNode();
    #endif
  };
  std::signal(SIGINT, handle);

  if(node->isInit())
    node->run();
  else
    ROS_ERROR("Rolling map node failed to initialize.");

  return 0;
}
