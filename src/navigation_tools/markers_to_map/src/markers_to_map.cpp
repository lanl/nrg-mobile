#include "markers_to_map.h"

namespace markers_to_map
{

MarkersToMap::MarkersToMap() :
  n("~"),
  listener(),
  mapFrame(""),
  robotFrame(""),
  resolution(0.0),
  width(0.0),
  height(0.0),
  rate(0.0),
  markerInit(false)
{
  // Get parameters from ROS parameter server.
  std::string markerTopic;
  std::string mapTopic;
  std::string poseTopic;

  if(!n.getParam("marker_topic", markerTopic))
  {
    ROS_ERROR_STREAM("MarkersToMap: Failed to load marker_topic param. Setting to default: '/occupied_markers'");
    markerTopic = "/occupied_markers";
  }
  if(!n.getParam("map_topic", mapTopic))
  {
    ROS_ERROR_STREAM("MarkersToMap: Failed to load map_topic param. Setting to default: '/projected_map'");
    mapTopic = "/projected_map";
  }
  if(!n.getParam("pose_topic", poseTopic))
  {
    ROS_ERROR_STREAM("MarkersToMap: Failed to load pose_topic param. Setting to default: '/robot_pose'");
    poseTopic = "/robot_pose";
  }
  if(!n.getParam("map_frame", mapFrame))
  {
    ROS_ERROR_STREAM("MarkersToMap: Failed to load map_frame param. Setting to default: 'map'");
    mapFrame = "map";
  }
  if(!n.getParam("robot_frame", robotFrame))
  {
    ROS_ERROR_STREAM("MarkersToMap: Failed to load robot_frame param. Setting to default: 'base_link'");
    robotFrame = "base_link";
  }
  if(!n.getParam("map_resolution", resolution))
  {
    ROS_ERROR_STREAM("MarkersToMap: Failed to load map_resolution param. Setting to default: 0.5");
    resolution = 0.5;
  }
  if(!n.getParam("map_width", width))
  {
    ROS_ERROR_STREAM("MarkersToMap: Failed to load map_width param. Setting to default: 5.0");
    width = 5.0;
  }
  if(!n.getParam("map_height", height))
  {
    ROS_ERROR_STREAM("MarkersToMap: Failed to load map_height param. Setting to default: 5.0");
    height = 5.0;
  }
  if(!n.getParam("rate", rate))
  {
    ROS_ERROR_STREAM("MarkersToMap: Failed to load rate param. Setting to default: 10.0");
    rate = 10.0;
  }

  // Subscribe to markers and robot pose
  markerSub = n.subscribe(markerTopic,1,&MarkersToMap::markerCb,this);
  poseSub = n.subscribe(poseTopic,1,&MarkersToMap::poseCb,this);

  // advertise map
  mapPub = n.advertise<nav_msgs::OccupancyGrid>(mapTopic,1,true);

  ROS_INFO("MarkersToMap: Markers to Map node initialized");
} // MarkersToMap

MarkersToMap::~MarkersToMap()
{
  // Do nothing
}

void MarkersToMap::markerCb(const visualization_msgs::MarkerArray msg)
{
  if(msg.markers.size() <= 0)
  {
    ROS_ERROR("MarkersToMap: Received blank marker array. Cannot make costmap");
    return;
  }
  markersMutex.lock();
  for(int i = 0; i < msg.markers.size(); i ++)
  {
    // if marker id is already used, delete the old one
    MarkerMap::iterator it;
    it = markers.find(msg.markers[i].id);
    if(it != markers.end())
    {
      markers.erase(it);
    }

    // If this is an add or modify, add the marker
    if(visualization_msgs::Marker::ADD == msg.markers[i].action)
    {
      markers.insert(std::make_pair(msg.markers[i].id,msg.markers[i]));
    }
  }
  markersMutex.unlock();
  markerInit = true; 
}

void MarkersToMap::poseCb(const geometry_msgs::PoseStamped msg)
{
  robotPose = msg;
}

void MarkersToMap::publishMap()
{
  // Make sure we have a marker array
  if(!markerInit)
  {
    ROS_ERROR("MarkersToMap: Cannot publish map. Waiting for marker array");
    return;
  }

  // Store off current robot pose to make map around it
  geometry_msgs::PoseStamped currentPose = robotPose;

  // Convert robot pose to map frame
  if(!toFrame(currentPose, mapFrame))
  {
    ROS_ERROR("MarkersToMap: Could not convert robot pose to map frame. Cannot make costmap");
    return;
  }

  // Lock mutex before using markers array
  markersMutex.lock();

  // Create occupancy grid and initialize structure
  nav_msgs::OccupancyGrid grid;
  grid.header.stamp = ros::Time::now();
  grid.header.frame_id = mapFrame;
  grid.info.resolution = resolution;
  grid.info.width = width/resolution;
  grid.info.height = height/resolution;
  grid.info.origin.orientation.w = 1.0;
  grid.info.origin.position.x = currentPose.pose.position.x - grid.info.width*resolution/2.0; // Robot should be in middle of map
  grid.info.origin.position.y = currentPose.pose.position.y - grid.info.height*resolution/2.0;
  grid.data.resize(grid.info.height*grid.info.width,0);

  // Fill occupancy data based on marker array
  for(MarkerMap::iterator it = markers.begin(); it!=markers.end(); ++it)
  {
    for(int i = 0; i < (*it).second.points.size(); i++)
    {
      geometry_msgs::PoseStamped pointPose;
      pointPose.header = (*it).second.header;
      pointPose.pose.orientation.w = 1.0;
      pointPose.pose.position.x = (*it).second.points[i].x;
      pointPose.pose.position.y = (*it).second.points[i].y;
      if(toFrame(pointPose,mapFrame))
      {
        int column = (pointPose.pose.position.x - grid.info.origin.position.x)/resolution;
        int row = (pointPose.pose.position.y - grid.info.origin.position.y)/resolution;
        if(row >= 0 && column >= 0 && row < grid.info.height && column < grid.info.width)
        {
          grid.data[row*grid.info.width + column] = 100;
        }
      }
    }
  }

  // Unlock mutex after using markers array
  markersMutex.unlock();

  // cycle through map and get rid of lonely cells
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

  // publish the map
  mapPub.publish(grid);
}

bool MarkersToMap::isOccupied(int r, int c, const nav_msgs::OccupancyGrid &g)
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

bool MarkersToMap::toFrame(geometry_msgs::PoseStamped &pose, std::string frame)
{
  if(pose.header.frame_id.compare(frame) != 0)
  {
    tf::Stamped<tf::Pose> in, out;
    pose.header.stamp = ros::Time(0);
    tf::poseStampedMsgToTF(pose, in);
    try
    {
      if(listener.waitForTransform(frame, pose.header.frame_id, ros::Time(0), ros::Duration(1.0)))
      {
        listener.transformPose(frame, in, out);
        tf::poseStampedTFToMsg(out, pose);
        return true;
      }
      else
      {
        ROS_ERROR_STREAM("MarkersToMap: toFrame timed out. frame1: " << pose.header.frame_id << " frame2: " << frame);
        return false;
      }
    }
    catch(...)
    {
      ROS_ERROR_STREAM("MarkersToMap: exception in toFrame. frame1: " << pose.header.frame_id << " frame2: " << frame);

      return false;
    }
  }
  else
  {
    // Do nothing, the pose is already in the correct frame
    return true;
  }

}

void MarkersToMap::run()
{
  ros::Rate loopRate(rate);
  while(ros::ok())
  {
    publishMap();
    loopRate.sleep();
  }
}

} // namespace markers_to_map

// Main function initialized the node and creates the wall planner node instance
int main(int argc, char **argv)
{
  // Initialize node with name "markers_to_map"
  ros::init(argc, argv, "markers_to_map");

  // AsyncSpinner allows ROS functionality to execute when necessary
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Creates an instance of MarkersToMap
  markers_to_map::MarkersToMap mToM;

  // start publish loop
  mToM.run();

  return 0; // success
} // main
