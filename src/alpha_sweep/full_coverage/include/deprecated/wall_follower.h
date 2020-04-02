#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include "full_coverage/BoolReq.h"
#include "full_coverage/FollowStatus.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"

class WallFollower
{
public:
  /* Wall follower constructor
   * @param nh         ROS Node handle
   * @param wallD      Desired distnace to maintain between robot and wall
   * @param velTopic   ROS topic name for command velocity topic
   * @param scanTopic  ROS topic name for laser scan topic
   */
  WallFollower(ros::NodeHandle *nh, double wallD, std::string velTopic, std::string scanTopic, std::string bumpTopic);

  /* Wall follower destrcutor
   */
  ~WallFollower();

  void stopRobot();
  void updatePose();
  void publishStatus();

private:
  void publishVelocity();
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void bumpCallback(const std_msgs::Bool::ConstPtr& msg);
  bool startCallback(full_coverage::BoolReq::Request &req, full_coverage::BoolReq::Response &res);
  double getRotPercent();
  double getLinPercent();

  ros::NodeHandle *n;              // ROS nodehandle
  tf::StampedTransform startPose;  // The pose of the robot at the start of a loop
  tf::StampedTransform robotPose;  // The most recent recoreded pose of the robot
  tf::TransformListener listener;  // TF Transform listener to listen to robot pose (odom->base_link transform)

  double wallDist;        // Desired distance to maintain between wall and robot
  double closestDist;     // Distance to the closest point in the laser scan
  double closestAngle;    // Angle, at which was measured the shortest distance
  double frontDist;       // Distance to closest laser point in front of robot
  bool bump;

  bool startCmd;          // True if client has requested wall follower to start

  ros::Publisher velPub;   // ROS Publisher for command velocity (Twist message)
  ros::Publisher statusPub;   // ROS Publisher for status
  ros::Publisher safeDrivePub;   // ROS Publisher for if it is safe to drive forward
  ros::Subscriber scanSub; // ROS Subscriber for laser scan.
  ros::Subscriber bumpSub; // ROS Subscriber for bumper hits.
  ros::ServiceServer startService; // ROS Service server used to start and stop the wall follower.

  full_coverage::FollowStatus state;
};
