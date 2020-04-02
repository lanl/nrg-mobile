#include "wall_follower.h"
#include <math.h>
#define PI 3.141592

WallFollower::WallFollower(ros::NodeHandle *nh, double wallD, std::string velTopic, std::string scanTopic, std::string bumpTopic) :
  n(nh),
  wallDist(wallD),
  closestDist(0.0),
  closestAngle(0.0),
  frontDist(0.0),
  bump(false),
  startCmd(false)
{
  state.status = full_coverage::FollowStatus::IDLE;
  velPub = n->advertise<geometry_msgs::Twist>(velTopic, 1);
  statusPub = n->advertise<full_coverage::FollowStatus>("/wall_follower/status", 1);
  safeDrivePub = n->advertise<std_msgs::Bool>("/wall_follower/safe_drive", 1);
  scanSub = n->subscribe(scanTopic, 1, &WallFollower::scanCallback, this);
  bumpSub = n->subscribe(bumpTopic, 1, &WallFollower::bumpCallback, this);
  startService = n->advertiseService("/wall_follower/start", &WallFollower::startCallback, this);
}

WallFollower::~WallFollower()
{
  stopRobot();
}

//Publisher
void WallFollower::publishVelocity()
{
  // Get appropriate linear and rotational velocity command
  geometry_msgs::Twist vel;
  vel.linear.x = getLinPercent();
  vel.linear.y = 0.0;
  vel.linear.z = 0.0;
  vel.angular.x = 0.0;
  vel.angular.y = 0.0;
  vel.angular.z = getRotPercent();

  // Switch to new state if we have succeeded in current state
  switch(state.status)
  {
    case full_coverage::FollowStatus::FINDING:
      // Move to approaching state when we are done turning to wall
      if(fabs(vel.angular.z) < 1.0)
        state.status = full_coverage::FollowStatus::APPROACHING;
      break;
    case full_coverage::FollowStatus::APPROACHING:
      // Move to following state and record start pose when we are near wall
      if(fabs(wallDist - closestDist) < 0.05)
      {
        startPose = robotPose;
        state.status = full_coverage::FollowStatus::FOLLOWING;
      }
      break;
    case full_coverage::FollowStatus::FOLLOWING:
      // Keep on keepin on (update pose function will stop the robot when it reaches the start pose)
      break;
    case full_coverage::FollowStatus::IDLE:
      // do nothing
      break;
    default:
      // this should not happen
      break;
  }

  // publish command velocity
  velPub.publish(vel);
}

double WallFollower::getLinPercent()
{
  double dDelta = closestDist - wallDist;
  if(full_coverage::FollowStatus::APPROACHING == state.status)
  {
    if(dDelta > 2.0)
      return 100.0;
    else if(dDelta > 1.0)
      return 20.0;
    else if(dDelta > 0.50)
      return 21.0*dDelta;
    else if(dDelta > 0.05)
      return 11.0;
    else if(dDelta > -0.05)
      return 0.0;
    else
      return -11.0;
  }
  else if(full_coverage::FollowStatus::FOLLOWING == state.status)
  {
    double aDelta = closestAngle + PI/2.0;
    if(dDelta < -0.04)
    {
      if(aDelta > 0.0)
        return 0.0;
      else
        return 11.0;
    }
    else
      return 11.0;
  }
  else
    return 0.0;
}

double WallFollower::getRotPercent()
{
  double aDelta = 0.0;
  if(full_coverage::FollowStatus::FOLLOWING == state.status)
  {
    double dDelta = closestDist - wallDist;
    aDelta = closestAngle + PI/2.0;
    //ROS_INFO_STREAM("Angle Min: " << closestAngle << " Angle Del: " << aDelta << " Dist del: " << dDelta);

    aDelta -= dDelta*1.5;
    //ROS_INFO_STREAM("Comp angle del: " << aDelta);
  }
  else
  {
    aDelta = closestAngle;
  }

  if(aDelta > M_PI/8.0)
    return 100.0;
  else if(aDelta > M_PI/16.0)
    return 25.0;
  else if(aDelta > M_PI/64.0)
    return 10.0;
  else if(aDelta > -M_PI/64.0)
    return 0.0;
  else if(aDelta > -M_PI/16.0)
    return -10.0;
  else if(aDelta > -M_PI/8.0)
    return -25.0;
  else
    return -100.0;
}

bool WallFollower::startCallback(full_coverage::BoolReq::Request &req, full_coverage::BoolReq::Response &res)
{
  startCmd = req.data;
  if(startCmd)
    state.status = full_coverage::FollowStatus::FINDING;
  else
    stopRobot();
}

void WallFollower::bumpCallback(const std_msgs::Bool::ConstPtr& msg)
{
  bump = msg->data;
}

void WallFollower::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  if(bump)
    startCmd = false;
  if(startCmd)
  {
    int size = msg->ranges.size();

    int minIndex = 0;
    int maxIndex = size-1;

    //Variables whith index of highest and lowest value in array.
    if(full_coverage::FollowStatus::FOLLOWING == state.status)
    {
      maxIndex = size*3/4;
    }

    //This cycle goes through array and finds minimum
    for(int i = minIndex; i < maxIndex; i++)
    {
      if ((msg->ranges[i] < msg->ranges[minIndex] && msg->ranges[i] > 0.0) || msg->ranges[minIndex] <= 0.0){
        minIndex = i;
      }
    }

    // Store angle and distance to closest point and front distance
    closestAngle = (minIndex-size/2)*msg->angle_increment;

    // adjust distance based on angle (might be closer to another part of the vehicle than the LIDAR point)
    if(full_coverage::FollowStatus::FOLLOWING == state.status && fabs(closestAngle + M_PI/2.0) < M_PI/4.0)
    {
      double tempDist = msg->ranges[minIndex];
      closestDist = cos(fabs(M_PI/2.0 + closestAngle))*tempDist;
      //ROS_INFO_STREAM_THROTTLE(1.0, "dist: " << tempDist << " adjusted: " << closestDist << " closestAngle: " << closestAngle << " cos: " << cos(fabs(M_PI/2.0 + closestAngle)));
    }
    else
    {
      closestDist = msg->ranges[minIndex];
    }
    frontDist = msg->ranges[size/2];

    //ROS_INFO_STREAM("Min angle: " << closestAngle << " Min Distance: " << closestDist << " Min index: " << minIndex);
    // publish command velocity
    publishVelocity();

    // Publish safe drive as false (no driving elsewhere while wall follower is in control
    std_msgs::Bool msg;
    msg.data = false;
    safeDrivePub.publish(msg);
  }
  else
  {
    std_msgs::Bool safe_msg;
    if(!bump)
    {
      double sd = 0.3; // safe distance to an obstacle (m)
      double rw = 0.5; // robot width
      double thetaMax = atan(rw/(2*sd)); // max laser theta that gives us obstacles in the safety zone (radians)
      int minIndex = (-thetaMax - msg->angle_min)/msg->angle_increment;
      int maxIndex = (thetaMax - msg->angle_min)/msg->angle_increment;
      if(minIndex < 0 || minIndex>=msg->ranges.size())
      {
        ROS_ERROR("Safe check: min index out of range.");
        minIndex = 0;
      }
      if(maxIndex < 0 || maxIndex>=msg->ranges.size())
      {
        ROS_ERROR("Safe check: max index out of range.");
        maxIndex = msg->ranges.size()-1;
      }

      //ROS_INFO_STREAM_THROTTLE(1.0,"minI: " << minIndex << " maxI: " << maxIndex << " thetaMax: " << thetaMax);

      int violationCount = 0;
      // Cycle through min to max index seeing if obstacles are in safety zone
      for(int i = minIndex; i< maxIndex; i++)
      {
        double thetaI = msg->angle_min + i*msg->angle_increment;
        double sdi = sd/cos(thetaI);
        //ROS_INFO_STREAM_THROTTLE(1.0, "sdi: " << sdi << " thetaI: " << thetaI);
        if(msg->ranges[i] > 0.0 && msg->ranges[i] < sdi)
          violationCount++;
      }

      if(violationCount >= 3)
        safe_msg.data = false;
      else
        safe_msg.data = true;
    }
    else
      safe_msg.data = false;
    safeDrivePub.publish(safe_msg);
  }
}

void WallFollower::stopRobot()
{
  // set start command to false and state to idle
  startCmd = false;
  state.status = full_coverage::FollowStatus::IDLE;

  // publish zero velocity
  geometry_msgs::Twist vel;
  vel.linear.x = 0.0;
  vel.linear.y = 0.0;
  vel.linear.z = 0.0;
  vel.angular.x = 0.0;
  vel.angular.y = 0.0;
  vel.angular.z = 0.0;

  velPub.publish(vel);
}

void WallFollower::updatePose()
{
  if(listener.waitForTransform("odom", "base_link", ros::Time(0),ros::Duration(1.0)))
  {
    listener.lookupTransform("odom", "base_link", ros::Time(0), robotPose);
    if(startCmd && full_coverage::FollowStatus::FOLLOWING == state.status)
    {
      if(fabs(robotPose.stamp_.toSec()-startPose.stamp_.toSec()) > 10.0)
      {
        if(sqrt(pow(robotPose.getOrigin().getX() - startPose.getOrigin().getX(),2)+pow(robotPose.getOrigin().getY() - startPose.getOrigin().getY(),2)) < 0.5)
        {
          stopRobot();
        }
      }
    }
  }
}

void WallFollower::publishStatus()
{
  statusPub.publish(state);
}

int main(int argc, char **argv)
{
  // Initialize node
  ros::init(argc, argv, "wall_follower");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(3);

  // Load params
  double wallD;
  std::string velTopic, scanTopic, bumpTopic;
  n.param<double>("wall_distance", wallD, 0.5);
  n.param<std::string>("vel_topic", velTopic, "/rosarnl_node/cmd_vel");
  n.param<std::string>("bump_topic", bumpTopic, "/rosarnl_node/bumper_triggered");
  n.param<std::string>("laser_topic", scanTopic, "/scan");

  // Wall follower object
  WallFollower *wallFollower = new WallFollower(&n, wallD, velTopic, scanTopic, bumpTopic);
  spinner.start();

  ros::Rate loopRate(10.0);
  while(ros::ok())
  {
    wallFollower->updatePose();
    wallFollower->publishStatus();
    loopRate.sleep();
  }

  delete wallFollower;
  return 0;
}
