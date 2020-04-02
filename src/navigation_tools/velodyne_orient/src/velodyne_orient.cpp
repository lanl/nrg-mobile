#include <ros/ros.h>
#include <string.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h> 
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/server.h>
#include <velodyne_orient/velodyne_orientConfig.h>
#include <mutex>
#include <atomic>

namespace velodyne_orient
{

class VelodyneOrient
{
public:
  VelodyneOrient();
  void pcCallback(const sensor_msgs::PointCloud2::ConstPtr& pc);
  void pcCallback2(const sensor_msgs::PointCloud2::ConstPtr& pc);
  void reconfigureCallback(velodyne_orientConfig &config, uint32_t level);
  void run();

private:
  void sendTransform();
  void findTransform();
  void addToAverage(const tf::StampedTransform &t);

  std::mutex tMutex;
  std::mutex cMutex;

  ros::NodeHandle n;
  ros::AsyncSpinner spinner;
  tf::TransformListener listener;
  tf::TransformBroadcaster broadcaster;
  tf::StampedTransform transform;
  tf::StampedTransform averageTransform;
  tf::StampedTransform mapClipTransform;
  sensor_msgs::PointCloud2 pointCloud;
  bool hasPC;
  ros::Time lastPublish;
  std::atomic<bool> useAverage;

  ros::Subscriber pcSub;
  ros::Subscriber pcSub2;
  ros::Publisher pcPub;
  ros::Publisher groundPub;

  bool optomize;
  int maxIt;
  double distThresh;
  bool setAngle;
  double angleThresh;
  bool setAxis;
  double maxDistFromZ0;
  bool publishGround;
  double cRate;
  double pRate;
  int maxAttempts;
};

VelodyneOrient::VelodyneOrient() :
  n("~"),
  spinner(3),
  hasPC(false),
  useAverage(false),
  cRate(10.0),
  pRate(10.0)
{
  pcSub = n.subscribe("/velodyne_points",1, &VelodyneOrient::pcCallback, this);
  pcSub2 = n.subscribe("/velodyne_points",1, &VelodyneOrient::pcCallback2, this);
  pcPub = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points_adjusted", 1, false);
  groundPub = n.advertise<sensor_msgs::PointCloud2>("ground", 1, false);

  tMutex.lock();
  transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
  tf::Quaternion q;
  q.setRPY(0,0,0);
  transform.setRotation(q);
  transform.child_frame_id_ = "velodyne_adjusted";
  transform.frame_id_ = "velodyne";
  averageTransform = transform;
  tMutex.unlock();
  mapClipTransform.setRotation(q);
  mapClipTransform.child_frame_id_ = "map_clip";
  mapClipTransform.frame_id_ = "map";
  mapClipTransform.setOrigin(tf::Vector3(0.0,0.0,0.027));

  lastPublish = ros::Time::now();

  sendTransform();

  spinner.start(); 
}

void VelodyneOrient::reconfigureCallback(velodyne_orientConfig &config, uint32_t level)
{
  optomize = config.optomize;
  maxIt = config.max_iterations;
  distThresh = config.dist_thresh;
  setAngle = config.set_angle;
  angleThresh = config.angle_thresh;
  setAxis = config.set_axis;
  maxDistFromZ0 = config.max_dist_from_z_0;
  publishGround = config.publish_ground;
  cRate = config.calc_rate;
  pRate = config.publish_rate;
  maxAttempts = config.max_attempts;
}

void VelodyneOrient::pcCallback(const sensor_msgs::PointCloud2::ConstPtr& pc)
{
  cMutex.lock();
  pointCloud = *pc;
  cMutex.unlock();
  hasPC = true;
  sendTransform();
}

void VelodyneOrient::findTransform()
{
  cMutex.lock();
  sensor_msgs::PointCloud2 pc = pointCloud;
  cMutex.unlock();
  sensor_msgs::PointCloud2 rosCloud;
  sensor_msgs::PointCloud2 rosGround;
  bool found = false;

  // try filtering out the ground frame
  try
  {
    // transform to robot frame
    if(listener.waitForTransform("base_link",pc.header.frame_id,pc.header.stamp,ros::Duration(0.3)))
    {
      if(!pcl_ros::transformPointCloud("base_link",pc,rosCloud,listener))
        ROS_ERROR_THROTTLE(1.0, "VelodyneOrient: failed to transform cloud to robot frame.");
    }
    else
      ROS_ERROR_THROTTLE(1.0, "VelodyneOrient: waitfortranform(): failed to transform cloud to robot frame.");

    // Convert to PCL and frame_id frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSpare(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(rosCloud,*cloud);
    pcl::fromROSMsg(rosCloud,*cloudSpare);

    // set up filter vars
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(optomize);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIt);
    seg.setDistanceThreshold(distThresh);
    if(setAngle)
      seg.setEpsAngle(angleThresh);
    if(setAxis)
      seg.setAxis(Eigen::Vector3f(0,0,1));
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    // keep segmenting till we find the ground
    int count = 0;
    while(count < maxAttempts && !found)
    {
      if(count%2 == 0)
        seg.setInputCloud(cloud);
      else
        seg.setInputCloud(cloudSpare);
      seg.segment(*inliers, *coefficients);

      if(inliers->indices.size() > 0)
      {
        //if(fabs(coefficients->values.at(3)) < maxDistFromZ0 || maxDistFromZ0 < 0.0001)

        // extract ground plane
        extract.setNegative(false);
        extract.setIndices(inliers);
        pcl::PointXYZ min,max;
        if(count%2 == 0)
        {
          extract.setInputCloud(cloud);
          extract.filter(*cloudSpare);
          pcl::getMinMax3D(*cloudSpare, min, max);
        }
        else
        {
          extract.setInputCloud(cloudSpare);
          extract.filter(*cloud);
          pcl::getMinMax3D(*cloud, min, max);
        }
        //ROS_INFO_STREAM("MinX: " << min.x << " MaxX: " << max.x);
        //ROS_INFO_STREAM("MinY: " << min.y << " MaxY: " << max.y);
        //ROS_INFO_STREAM("MinZ: " << min.z << " MaxZ: " << max.z);
        //ROS_INFO_STREAM("Coefficients: 0: " << coefficients->values.at(0) << " 1: " << coefficients->values.at(1) << " 2: " << coefficients->values.at(2) << " 3: " << coefficients->values.at(3));
        //ROS_INFO_STREAM(" ");

        if(min.z < -maxDistFromZ0 || max.z > maxDistFromZ0)
        {
          // keep everything but the plane that was found
          extract.setNegative(true);
          extract.setIndices(inliers);
          if(count%2 == 0)
          {
            extract.setInputCloud(cloud);
            extract.filter(*cloudSpare);
          }
          else
          {
            extract.setInputCloud(cloudSpare);
            extract.filter(*cloud);
          }
        }
        else
          found = true;
      }
      else
      {
        ROS_ERROR_THROTTLE(1.0, "VelodyneOrient: found no inliers.");
      }
      count++;
    }

    if(!found)
    {
       ROS_ERROR_THROTTLE(1.0, "VelodyneOrient: Failed to find ground plane in range.");
       useAverage = true;
    }
    else
    {
      // adjust velodyne tranform
      Eigen::Vector3d baseLinkZ(0.0,0.0,1.0);
      Eigen::Vector3d planeZ(coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(2));
      Eigen::Quaterniond eQuat = Eigen::Quaterniond().setFromTwoVectors(planeZ,baseLinkZ);
      tf::Quaternion quat(eQuat.x(), eQuat.y(), eQuat.z(), eQuat.w());
      tMutex.lock();
      transform.setRotation(quat);
      transform.setOrigin(tf::Vector3(0.0,0.0,coefficients->values.at(3)));
      tMutex.unlock();
      useAverage = false;

      // convert back to sensor message and publish
      if(publishGround)
      {
        if(count%2 != 0)
          pcl::toROSMsg(*cloudSpare,rosGround);
        else
          pcl::toROSMsg(*cloud,rosGround);

        // transform to sensor frame
        if(listener.waitForTransform("velodyne","base_link",pc.header.stamp,ros::Duration(0.3)))
        {
          if(!pcl_ros::transformPointCloud("velodyne",rosGround,rosGround,listener))
            ROS_ERROR_THROTTLE(1.0, "VelodyneOrient: failed to transform ground cloud to sensor frame. Publishing in working frame.");
        }
        else
          ROS_ERROR_THROTTLE(1.0, "VelodyneOrient: waitfortransform(): failed to transform ground cloud to sensor frame. Publishing in working frame.");
      }
    }
  }
  catch(...)
  {
    ROS_ERROR_THROTTLE(1.0, "velodyne_orient: Failed to manipulate pointcloud");
  }

  if(found && publishGround)
    groundPub.publish(rosGround);
}

void VelodyneOrient::pcCallback2(const sensor_msgs::PointCloud2::ConstPtr& pc)
{
  if(ros::Time::now().toSec() - lastPublish.toSec() > 1/pRate)
  {
    lastPublish = ros::Time::now();
    sensor_msgs::PointCloud2 temp = *pc;
    temp.header.frame_id = "velodyne_adjusted";
    pcPub.publish(temp);
  }
}

void VelodyneOrient::sendTransform()
{
  tf::StampedTransform temp;
  tMutex.lock();
  temp = transform;
  tMutex.unlock();
  temp.stamp_ = ros::Time::now();
  addToAverage(temp);
  if(!useAverage)
    broadcaster.sendTransform(temp);
  else
    broadcaster.sendTransform(averageTransform);
  mapClipTransform.stamp_ = ros::Time::now();
  broadcaster.sendTransform(mapClipTransform);
}

void VelodyneOrient::addToAverage(const tf::StampedTransform &t)
{
  averageTransform.stamp_ = ros::Time::now();
  tf::Quaternion quat = averageTransform.getRotation();
  tf::Vector3 vect = averageTransform.getOrigin();
  tf::Quaternion tquat = t.getRotation();
  tf::Vector3 tvect = t.getOrigin();
  quat.setX(quat.getX() - (quat.getX()/50) + (tquat.getX()/50));
  quat.setY(quat.getY() - (quat.getY()/50) + (tquat.getY()/50));
  quat.setZ(quat.getZ() - (quat.getZ()/50) + (tquat.getZ()/50));
  quat.setW(quat.getW() - (quat.getW()/50) + (tquat.getW()/50));
  vect.setX(vect.getX() - (vect.getX()/50) + (tvect.getX()/50));
  vect.setY(vect.getY() - (vect.getY()/50) + (tvect.getY()/50));
  vect.setZ(vect.getZ() - (vect.getZ()/50) + (tvect.getZ()/50));
  averageTransform.setRotation(quat);
  averageTransform.setOrigin(vect);
}

void VelodyneOrient::run()
{
  double oldRate = cRate;
  ros::Rate loopRate(cRate);
  while(ros::ok())
  {
    if(fabs(cRate-oldRate) > 0.01)
    {
      oldRate = cRate;
      loopRate = ros::Rate(cRate);
    }
    if(hasPC)
    {
      findTransform();
    }
    loopRate.sleep();
  }
}

} // end ns velodyne_orient

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velodyne_orient");

  velodyne_orient::VelodyneOrient velOrient;

  dynamic_reconfigure::Server<velodyne_orient::velodyne_orientConfig> server;
  dynamic_reconfigure::Server<velodyne_orient::velodyne_orientConfig>::CallbackType f;

  f = boost::bind(&velodyne_orient::VelodyneOrient::reconfigureCallback,&velOrient, _1, _2);
  server.setCallback(f);

  velOrient.run();
  
  return 0;
}

