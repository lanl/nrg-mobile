#ifndef _ROLLING_MAP_NODE_H_
#define _ROLLING_MAP_NODE_H_

#include "ros/ros.h"
#include "rolling_map.h"
#include "tf/transform_listener.h"
#include "pcl_ros/point_cloud.h"
#include "std_srvs/Empty.h"
#include "nav_msgs/OccupancyGrid.h"
#include "rolling_map/Box.h"


namespace rolling_map
{
struct MapParams
{
  std::string pc_topic;
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
};

class RollingMapNode 
{
private:
  ros::NodeHandle n;
  ros::AsyncSpinner spinner;
  tf::TransformListener listener;
  bool init;

  // Parameters
  MapParams param;

  // Listen for point clouds
  bool hasData;
  ros::Subscriber pcSub;
  void pcCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg);

  // Publish visual data and map
  ros::Publisher markerPub;
  ros::Publisher mapPub;
  ros::Publisher readyPub;
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

};

} // namespace rolling_map

#endif // _ROLLING_MAP_NODE_H_
