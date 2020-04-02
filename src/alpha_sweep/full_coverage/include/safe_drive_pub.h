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
