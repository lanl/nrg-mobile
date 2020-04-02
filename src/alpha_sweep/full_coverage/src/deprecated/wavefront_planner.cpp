#include "wavefront_planner.h"

namespace full_coverage
{

WavefrontPlanner::WavefrontPlanner() :
  n("~"),
  isInitialized(false),
  poseFrame("")
{
  // Create service client that will get map information from the coarse grid converter
  ros::ServiceClient grid_client = n.serviceClient<full_coverage::GetCoarseGrid>("/coarse_grid_converter/get_coarse_grid");
  
  // Wait until the coarse grid converter publishes the get_coarse_grid service
  bool serviceExists = false;
  while(!serviceExists)
  {
    serviceExists = grid_client.waitForExistence(ros::Duration(5));
  }
  
  // Use a service call to retrieve map data from get_coarse_grid service
  full_coverage::GetCoarseGrid grid_srv;  // stores service call request and response data
  grid_srv.request.print_map = false;     // do not print map to the terminal
  bool gridReceived = false;
  while(!gridReceived) // we cannot continue until we have the map data
  {
    if (grid_client.call(grid_srv)) // grid service call returns false if unsuccessful
    {
      gridReceived = true;
    }
    else
    {
      ROS_INFO_THROTTLE(5, "Wavefront Planner: Waiting to receive coarse grid.");
    }
  }
  
  // Use the ROS parameter server to check what the occupancy threshold and robot cell dimensions are.
  int occThreshold;
  if(!n.getParam("/coarse_grid_converter/GridConvertParams/occupancy_threshold", occThreshold))
  {
    ROS_ERROR_STREAM("Wavefront Planner: Could not get occupancy threshold from param server. Setting to default 10%");
    occThreshold = 10;
  }
  int robot_width;
  int robot_height;
  if(!n.getParam("/wavefront_planner/RobotParams/cell_width", robot_width) ||
     !n.getParam("/wavefront_planner/RobotParams/cell_height", robot_height))
  {
    ROS_ERROR_STREAM("Wavefront Planner: Could not get robot width or height. Setting to defauly (3cellsx3cells");
    robot_width = 3;
    robot_height = 3;
  }
  
  // Initialize wave and send it the map data
  if(wave.initialize(grid_srv.response.cell_grid, occThreshold, robot_width, robot_height))
  {
    // Advertise plan service now that we have the data that we need to plan
    planService = n.advertiseService("get_nav_plan", &WavefrontPlanner::getPlanCallback, this);
    neighborService = n.advertiseService("get_empty_neighbor", &WavefrontPlanner::getEmptyNeighborCallback, this);
    
    // Node is isInitialized
    isInitialized = true;
    
    ROS_INFO("Wavefront Planner: Grid received. Wavefront planner initialized");
  }
  else
  {
    ROS_ERROR("Wavefront Planner: Failed to initialize. Could not initialize wave.");
  }
} // WavefrontPlanner

WavefrontPlanner::~WavefrontPlanner()
{
  // Do nothing
}

bool WavefrontPlanner::planCallback(full_coverage::GenerateNavPlan::Request &req, full_coverage::GenerateNavPlan::Response &res)
{
  // Only respond that start is occupied if it is
  res.start_invalid = false;
  
  // Do nothing if the planner was not initialized correctly
  if(!isInitialized)
  {
    ROS_ERROR("Wavefront Planner: Cannot get plan before successful initialization.");
    return false;
  }
  
  // Set pose frame
  poseFrame = req.start_pose.header.frame_id;
  
  // If we are replanning, reset the wave data for the replan cells and reinitialize the wave so that a new wavefront will be generated
  if(req.replan_poses.size()!=0)
  {
    wave.resetCells(detachHeaders(req.replan_poses));
    wave.reinit();
  }
  
  // Fill wavefront grid so that a plan can be formed
  if(!wave.fill(req.start_pose.pose))
  {
    ROS_ERROR("Wavefront Planner: Cannot get navigation plan. Failed to fill wave.");
    res.start_invalid = true;
    return false;    
  }
  
  // Determine coverage plan using wavefront information
  std::vector<geometry_msgs::Pose> plan;
  if(!wave.getPlan(plan))
    return false;
  
  // Store plan in response variable and return true
  res.plan_poses = attachHeaders(plan);
  return true;
} // getPlanCallback

bool WavefrontPlanner::getPlanCallback(full_coverage::GetNavPlan::Request &req, full_coverage::GetNavPlan::Response &res)
{

}

std::vector<geometry_msgs::PoseStamped> WavefrontPlanner::attachHeaders(std::vector<geometry_msgs::Pose> poses)
{
  std::vector<geometry_msgs::PoseStamped> stampedPoses;
  for(int i = 0; i<poses.size(); i++)
  {
    geometry_msgs::PoseStamped temp;
    temp.header.frame_id = poseFrame;
    temp.header.seq = i;
    temp.header.stamp = ros::Time::now();
    temp.pose = poses[i];
    stampedPoses.push_back(temp);
  }
  return stampedPoses;
}

std::vector<geometry_msgs::Pose> WavefrontPlanner::detachHeaders(std::vector<geometry_msgs::PoseStamped> stampedPoses)
{
  std::vector<geometry_msgs::Pose> poses;
  for(int i = 0; i<stampedPoses.size(); i++)
  {
    poses.push_back(stampedPoses[i].pose);
  }
  return poses;
}

bool WavefrontPlanner::getEmptyNeighborCallback(full_coverage::GetEmptyNeighbor::Request &req, full_coverage::GetEmptyNeighbor::Response &res)
{
  geometry_msgs::PoseStamped neighbor = req.pose;
  wave.findEmptyNeighbor(neighbor.pose);
  res.neighbor = neighbor;
  return true;
}

} // namespace full_coverage

// Main function initialized the node and creates the wavefrontplanner instance
int main(int argc, char **argv)
{
  // Initialize node with name "wavefront_planner"
  ros::init(argc, argv, "wavefront_planner");

  // AsyncSpinner allows ROS functionality to execute when necessary
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Creates an instance of WavefrontPlanner
  full_coverage::WavefrontPlanner* wf = new full_coverage::WavefrontPlanner();

  // Do not kill the node until ROS shutdown so that plan service remains available
  ros::waitForShutdown();

  return 0; // success
} // main
