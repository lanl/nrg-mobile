/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date September 2016
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <sstream>
#include "../include/sweep_gui/qnode.hpp"
//#include <rosaria/WheelLightsEnum.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace sweep_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode() :
  n(),
  spinner(3),
  listener(),
  planClient("wall_planner_node/plan",true),
  executeClient("cover_map/execute",true),
  publishReady(false),
  turnSuccess(true),
  firstPose(true),
  undockExec(false),
  secondAttempt(false),
  isDocked(false),
  isEstopped(false),
  safeDrive(true),
  safeTurn(true),
  mappingReady(false),
  mRes(0.0),
  mWidth(0),
  mHeight(0),
  useCostmap(true),
  mapTopic(""),
  gotMap(false),
  mFrame(""),
  debug(false),
  status(STATUS_INITIALIZING),
  preStopStatus(STATUS_INITIALIZING),
  saveCount(0)
{
  // set dock pose
  dockPose.header.frame_id = "dock";
  dockPose.pose.position.x = 0.0;
  dockPose.pose.position.y = 0.0;
  dockPose.pose.position.z = 0.0;
  dockPose.pose.orientation.x = 0.0;
  dockPose.pose.orientation.y = 0.0;
  dockPose.pose.orientation.z = 0.0;
  dockPose.pose.orientation.w = 1.0;

  // make sure pose starts as valid pose
  mPose.orientation.w = 1.0;
}

QNode::~QNode()
{
  // do nothing
}

void QNode::sendShutdown()
{
  // Turn joystick back on since we lose control via GUI
  std_msgs::Bool bm;
  bm.data = true;
  joyEnablePub.publish(bm);

  // Now advertise shutdown to kill comptuer side ROS nodes
  std::cout << "Advertising shutdown" << std::endl;
  std_msgs::Empty msg;
  shutdownPub.publish(msg);
}

bool QNode::init() {
  // set up ros infrastcuture
  spinner.start();
  bool clientsValid = planClient.waitForServer(ros::Duration(50.0));
  clientsValid = clientsValid && executeClient.waitForServer(ros::Duration(10.0));
  if(!clientsValid)
  {
    info("Could not connect to action servers.",true);
    return false;
  }
  visitClient = n.serviceClient<full_coverage::Visit>("wall_planner_node/visit");

  // load param data
  if(!n.getParam("/Robot/width",robotWidth) ||
     !n.getParam("/Robot/length", robotLength) ||
     !n.getParam("/Sensor/width", sensorW) ||
     !n.getParam("/Sensor/length", sensorL) ||
     !n.getParam("/Sensor/x", sensorX) ||
     !n.getParam("/Sensor/y", sensorY) ||
     !n.getParam("/GUI/is_costmap", useCostmap) ||
     !n.getParam("/GUI/map_topic", mapTopic))
  {
    info("Could not load ROS parameters.",true);
    return false;
  }
  n.param("/debug", debug, false);

  // ros communications
  shutdownPub = n.advertise<std_msgs::Empty>("/ros_shutdown",1,this);
  //oresetClient = n.serviceClient<std_srvs::Empty>("/octomap_reset/reset");
  gresetPub = n.advertise<std_msgs::Int32>("/reset_gmapping",1,true);
  //wheelLightsPub = n.advertise<rosaria::WheelLightsEnum>("/rosaria/wheel_lights",1,this);
  gresetSub = n.subscribe("/reset_gmapping",1,&QNode::gresetCallback, this); // this is necessary to unlatch our latched message
  poseSub = n.subscribe("/robot_pose", 1, &QNode::poseCallback, this);
  targetSub = n.subscribe("/cover_map/target", 1, &QNode::targetCallback, this);
  navPoseSub = n.subscribe("/move_base/current_goal", 1, &QNode::navPoseCallback, this);
  // estopSub = n.subscribe("/rosaria/estop_pressed",1,&QNode::estopCallback,this);
  // dockSub = n.subscribe("/rosaria/battery_recharge_state",1,&QNode::dockCallback,this);
  mapClient = n.subscribe(mapTopic,1,&QNode::mapCallback, this);
  if(useCostmap)
    mapUpdatesClient = n.subscribe(mapTopic + "_updates",1,&QNode::mapUpdatesCallback, this);
  // joyEnablePub = n.advertise<std_msgs::Bool>("/rosaria/enable_joy",1);
  eraseMemoryClient = n.serviceClient<std_srvs::Empty>("/wall_planner_node/clear");
  if(debug)
    debugGridSub = n.subscribe("/debug_grid",1,&QNode::debugCallback, this);
  sdSub = n.subscribe("/safe_drive_pub/safe_drive",1,&QNode::sdCallback, this);
  stSub = n.subscribe("/safe_drive_pub/safe_turn",1,&QNode::stCallback, this);
  mappingReadySub = n.subscribe("rolling_map_node/ready",1,&QNode::mappingReadyCallback, this);

  while(!mappingReady)
  {
    ROS_ERROR_THROTTLE(1.0,"QNode: Waiting for 3d mapping data.");
    ros::Duration(0.1).sleep();
  }
  mappingReadySub.shutdown();

  publishReady = true;

  lastSent = ros::Time::now() - ros::Duration(0.2);
  lastMap = ros::Time::now() - ros::Duration(1.0);
  changeStatus(STATUS_IDLE);
  preStopStatus = STATUS_IDLE;

  return true;
}

void QNode::initRosPubs()
{
  while(!publishReady)
  {
    ROS_ERROR_THROTTLE(1.0,"QNode: Waiting for publishers to be initialized.");
    ros::Duration(0.1).sleep();
  }

  std_msgs::Int32 msg;
  msg.data = 0;
  gresetPub.publish(msg);

  // start with joystick disabled
  std_msgs::Bool bm;
  bm.data = false;
  joyEnablePub.publish(bm);
}

void QNode::mappingReadyCallback(const std_msgs::Bool msg)
{
  mappingReady = msg.data;
}

void QNode::dockCallback(const std_msgs::Int8 msg)
{
  if(msg.data != 0 && msg.data != -1)
  {
    isDocked = true;
    sendStopRobot();
    changeStatus(STATUS_DOCKED);
  }
  else
    isDocked = false;
}

void QNode::sdCallback(const std_msgs::Bool safe)
{
  if(safe.data!=safeDrive)
  {
    Q_EMIT newSafeD(safe.data);
    safeDrive = safe.data;
  }
}

void QNode::stCallback(const std_msgs::Bool safe)
{
  if(safe.data!=safeTurn)
  {
    Q_EMIT newSafeT(safe.data);
    safeTurn = safe.data;
  }
}

void QNode::debugCallback(full_coverage::DebugGrid msg)
{
  debugMsg = msg;
  Q_EMIT newDebugMsg();
}

full_coverage::DebugGrid QNode::getDebugGrid()
{
  return debugMsg;
}

void QNode::gresetCallback(std_msgs::Int32 msg)
{
  int reset = msg.data;
  if(reset == 2)
  {
    std_msgs::Int32 temp;
    temp.data = 0;
    gresetPub.publish(temp);
  }
}

// callback for estop state
void QNode::estopCallback(const std_msgs::Bool pressed)
{
  isEstopped = pressed.data;
  if(STATUS_ESTOPPED != status && pressed.data)
  {
    clearRobot(false,STATUS_ESTOPPED);
  }
  else if(STATUS_ESTOPPED == status && !pressed.data)
  {
    clearRobot(true);
  }
}

// callback function for robot pose message
void QNode::poseCallback(geometry_msgs::PoseStamped pose)
{
  if(pose.header.frame_id.compare("") != 0)
  {
    if(firstPose)
    {
      initRosPubs();
      lastResetPose = pose;
      lastResetTime = ros::Time::now();
    }
    if(ros::Time::now().toSec() - lastSent.toSec() > 0.1)
    {
      lastSent = ros::Time::now();
      robotPose = pose;
      Q_EMIT updatePose();
    }
    firstPose = false;
  }
}

double QNode::getDist(geometry_msgs::PoseStamped &p1, geometry_msgs::PoseStamped &p2)
{
  if(p1.header.frame_id.compare(p2.header.frame_id) == 0)
  {
    return pow(pow(p2.pose.position.x - p1.pose.position.x,2) + 
               pow(p2.pose.position.y - p1.pose.position.y,2), 0.5);
  }
  else
  {
    ROS_ERROR_THROTTLE(1.0, "Sweep GUI: Cannot get distance. Poses are in different frames.");
    return 0.0;
  }
}

// called by main window when updatePose signal is emmitted
geometry_msgs::Pose QNode::getPose()
{
  while(firstPose)
  {
    ROS_ERROR_THROTTLE(1.0, "QNode: waiting for robot pose.");
    ros::Duration(0.1).sleep();
  }

  geometry_msgs::PoseStamped rPose = robotPose;
  if(!toMapFrame(rPose))
  {
    ROS_ERROR_THROTTLE(1.0, "QNode: Failed to convert robot pose to map frame.");
  }
  return rPose.pose;
}

// called from update map function to update the pose of the dock
geometry_msgs::Pose QNode::getDockPose()
{
  geometry_msgs::PoseStamped dPose = dockPose;
  if(!toMapFrame(dPose))
  {
    ROS_ERROR_THROTTLE(1.0, "QNode: Failed to convert dock pose to map frame.");
  }
  return dPose.pose;
}

// callback function for pose target message
void QNode::targetCallback(geometry_msgs::PoseStamped pose)
{
  targetPose = pose;
  Q_EMIT updateTarget();
}

// called by main window when updateTarget signal is emitted
geometry_msgs::Pose QNode::getTarget()
{
  geometry_msgs::PoseStamped rPose = targetPose;
  if(!toMapFrame(rPose))
  {
    ROS_ERROR_THROTTLE(1.0, "QNode: Failed to convert target pose to map frame.");
  }
  return rPose.pose;
}

// callback function for nav pose message
void QNode::navPoseCallback(geometry_msgs::PoseStamped pose)
{
  navPose = pose;
  if(!toMapFrame(navPose))
  {
    ROS_ERROR_THROTTLE(1.0, "QNode: Failed to convert nav pose to map frame.");
  }
  Q_EMIT updateNavPose();
}

// called by main window when updateTarget signal is emitted
geometry_msgs::Pose QNode::getNavPose()
{
  return navPose.pose;
}

// callback function for new map published
void QNode::mapCallback(nav_msgs::OccupancyGrid grid)
{
  if(grid.info.width*grid.info.height != grid.data.size())
  {
    info("Received invalid occupancy grid. HXW != size", true);
    return;
  }
  if(grid.header.frame_id.compare(mFrame) != 0 && mFrame.compare("") != 0)
  {
    info("Received invalid occupancy grid. Not in map frame.", true);
    return;
  }

  mPose = grid.info.origin;
  mRes = grid.info.resolution;
  mWidth = grid.info.width;
  mHeight = grid.info.height;
  map = grid.data;
  mFrame = grid.header.frame_id;

  if(ros::Time::now().toSec() - lastMap.toSec() > 0.1)
  {
    lastMap = ros::Time::now();
    Q_EMIT newMap();
  }
  gotMap = true;
}

void QNode::mapUpdatesCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg)
{
  // If we don't have a full map yet, do nothing
  if(!gotMap)
  {
    return;
  }

  // Reject updates which have any out-of-bounds data.
  if(msg->x < 0 ||
     msg->y < 0 ||
     mWidth < msg->x + msg->width ||
     mHeight < msg->y + msg->height )
  {
    return;
  }

  // Copy the incoming data into map's data.
  for( size_t y = 0; y < msg->height; y++ )
  {
    memcpy( &map[ (msg->y + y) * mWidth + msg->x ],
            &msg->data[ y * msg->width ],
            msg->width );
  }

  // This is the refresh rate for the map
  if(ros::Time::now().toSec() - lastMap.toSec() > 0.2)
  {
    lastMap = ros::Time::now();
    Q_EMIT newMap();
  }
}

// called by main window when newMap signal is emitted
bool QNode::getMap(int &w, int &h, geometry_msgs::Pose &pose, double &res, std::vector<int8_t> &data)
{
  ros::Rate pause(5.0);
  ros::Time startTime = ros::Time::now();
  bool mapValid = mWidth != 0 && mHeight != 0 && map.size() != 0;
  while(!mapValid && ros::Time::now().toSec() - startTime.toSec() < 5.0)
  {
    ROS_INFO_THROTTLE(1.0, "QNode: Waiting for valid map");
    pause.sleep();
    mapValid = mWidth != 0 && mHeight != 0 && map.size() != 0;
  }
  if(!mapValid)
    return false;
  res = mRes;
  w = mWidth;
  h = mHeight;
  data.resize(map.size(),0);
  for(int i = 0; i < data.size(); i++)
  {
    if(map[i] < 100 && map[i] > 0)
      data[i] = 0;
    else
      data[i] = map[i];
  }
  pose = mPose;
  return true;
}

// Called by main window when joystick button is pressed
void QNode::enableJoy(bool enable)
{
  std_msgs::Bool msg;
  msg.data = enable;
  joyEnablePub.publish(msg);
  if(enable)
  {
    clearRobot(false,STATUS_JOY);
  }
  else
  {
    clearRobot(true);
  }
}

// tracks status (state) of node.
// Note allow clear will let the robot move from a stopped stated. Should only be true when called from revertStopStatus() function.
bool QNode::changeStatus(Status newStat, bool allowClear)
{
  if(newStat != status)
  {
    if(STATUS_UNCLEARED == status && !(allowClear || STATUS_ESTOPPED == newStat || STATUS_DOCKED == newStat))
    {
      Q_EMIT notCleared();
      return false;
    }
    if(STATUS_ESTOPPED == status && !allowClear && isEstopped)
    {
      if(STATUS_DOCKED == newStat)
      {
        isDocked = true;
      }
      else
      {
        std::cout << "tried to change status to " << newStat << " while e-stopped" << std::endl;
        Q_EMIT eStopped();
      }
      return false;
    }
    if(STATUS_JOY == status && !allowClear)
    {
      if(STATUS_ESTOPPED == newStat || STATUS_DOCKED == newStat)
      {
        // disable joystick
        std_msgs::Bool bm;
        bm.data = false;
        joyEnablePub.publish(bm);
      }
      else
      {
        Q_EMIT inJoy();
        return false;
      }
    }
    switch(newStat)
    {
      case STATUS_DOCKED:
        // update isdocked and fall through
        isDocked = true;
      case STATUS_INITIALIZING:
        // just change status
        // fall through
      case STATUS_IDLE:
        // just change status
        // fall through
      case STATUS_PLANNING:
        // just change status
        // fall through
      case STATUS_EXECUTING:
        // just change status
        // fall through
      case STATUS_DOCKING:
        // just change status
        // fall through
      case STATUS_UNDOCKING:
        // just change status
        // fall through
      case STATUS_PAUSED:
        // just change status
        status = newStat;
        break;
      case STATUS_UNCLEARED:
        status = STATUS_UNCLEARED;
        info("Robot is no longer cleared for motion", true);
        break;
      case STATUS_JOY:
        status = STATUS_JOY;
        info("Joystick is enabled", false);
        break;
      case STATUS_ESTOPPED:
        status = STATUS_ESTOPPED;
        info("Robot has been emergency stopped", true);
        break;
      //case STATUS_RESET_OCT:
      //  status = STATUS_RESET_OCT;
      //  info("Resetting octomap", false);
      //  break;
      default:
        status = NUM_STATUS;
        break;
    }
    //setWheelLights();
    Q_EMIT newStatus();
  }
  return true;
}

//void QNode::setWheelLights()
//{
//  rosaria::WheelLightsEnum msg;
//  switch(status)
//  {
//    case STATUS_ESTOPPED:
//      msg.status = rosaria::WheelLightsEnum::ESTOP;
//      break;
//    case STATUS_PLANNING:
//      msg.status = rosaria::WheelLightsEnum::PROCESSING;
//      break;
//    case STATUS_DOCKED:
//      msg.status = rosaria::WheelLightsEnum::CHARGING;
//      break;
//    case STATUS_INITIALIZING:
//      msg.status = rosaria::WheelLightsEnum::IDLE;
//      break;
//    case STATUS_IDLE:
//      msg.status = rosaria::WheelLightsEnum::IDLE;
//      break;
//    case STATUS_EXECUTING:
//      msg.status = rosaria::WheelLightsEnum::SWEEPING;
//      break;
//    case STATUS_DOCKING:
//      msg.status = rosaria::WheelLightsEnum::SWEEPING;
//      break;
//    case STATUS_UNDOCKING:
//      msg.status = rosaria::WheelLightsEnum::SWEEPING;
//      break;
//    case STATUS_PAUSED:
//      msg.status = rosaria::WheelLightsEnum:: PAUSED;
//      break;
//    case STATUS_UNCLEARED:
//      msg.status = rosaria::WheelLightsEnum::SENSOR;
//      break;
//    case STATUS_JOY:
//      msg.status = rosaria::WheelLightsEnum::JOYSTICK;
//      break;
//    case NUM_STATUS:
//      msg.status = rosaria::WheelLightsEnum::PROCESSING;
//      break;
//    default:
//      msg.status = rosaria::WheelLightsEnum::UNKNOWNSTATUS;
//      break;
//  }
//  wheelLightsPub.publish(msg);
//}

// called by main window when new status is emitted
Status QNode::getStatus()
{
  return status;
}

// get the robot width and height
void QNode::getRobotInfo(double &rw, double &rh, double &sw, double &sl, double &sx, double &sy)
{
  rw = robotWidth;
  rh = robotLength;
  sw = sensorW;
  sl = sensorL;
  sx = sensorX;
  sy = sensorY;
}

// Automatically called when planClient finishes a goal
void QNode::planDoneCallback(const actionlib::SimpleClientGoalState& state, const full_coverage::PlanResultConstPtr& res)
{
  if(actionlib::SimpleClientGoalState::SUCCEEDED == state.state_)
  {
    secondAttempt = false;
    turnSuccess = true;
    info(res->text, false);
    // success logic
    plan = res->plan;
    if (plan.poses.size() > 0) // we were successful and in map and sweep mode
    {
      Q_EMIT newPlan();
      startPlan(plan);
     
      // save the map png
      QString fn;
      fn.append("/home/demo/demoSave");
      fn.append(QString::number(saveCount));
      fn.append(".png");
      saveCount++;
      Q_EMIT saveRequest(fn);
    }
    else // we have explored the whole map
    {
      Q_EMIT execDone();
      changeStatus(STATUS_IDLE);
    }
  }
  else
  {
    // failure logic
    info(res->text, false);
    if(!secondAttempt)
    {
      secondAttempt = true;
      generatePlan(getPose());
    }
    else if(!turnSuccess)
    {
      execute(getPose());
    }
    else
    {
      Q_EMIT execDone();
      changeStatus(STATUS_IDLE);
    }
  }
}

// Called by Main Window when newPlan signal is received
bool QNode::getPlan(std::vector<geometry_msgs::Pose> &pl)
{
  pl.clear();

  std::vector<geometry_msgs::PoseStamped> tempPlan = plan.poses;
  bool success = true;
  for(int i = 0; i<tempPlan.size(); i++)
  {
    geometry_msgs::PoseStamped tempPose = tempPlan[i];
    if(!toMapFrame(tempPose))
    {
      ROS_ERROR_THROTTLE(1.0, "QNode: Failed to convert plan pose to map frame.");
      success = false;
    }
    pl.push_back(tempPose.pose);
  }
  if(success)
  {
    return true;
  }
  else
    return false;
}

// Automatically called when executeClient finsihes a goal
void QNode::executeDoneCallback(const actionlib::SimpleClientGoalState& state, const full_coverage::ExecuteResultConstPtr& res)
{
  if(res->type == full_coverage::ExecuteGoal::TURN)
  {
    turnSuccess = false;
    info(res->text, false);
    if(!generatePlan(startPose))
    {
      info("Failed to generate plan after TURN.", true);
      endExecution();
    }
  }
  else if(actionlib::SimpleClientGoalState::SUCCEEDED == state.state_)
  {
    // success logic
    info(res->text, false);
    if(STATUS_EXECUTING == status)
    {
      if(!generatePlan(getPose()))
      {
        info("Failed to generate new plan after execution.", true);
        endExecution();
      }
    }
    else if(full_coverage::ExecuteGoal::DOCK == res->type)
    {
      changeStatus(STATUS_DOCKED);
    }
    else if(full_coverage::ExecuteGoal::UNDOCK == res->type)
    {
      isDocked = false;
      revertStopStatus();
      if(undockExec)
      {
        undockExec = false;
        if(STATUS_IDLE == status)
          execute(getPose());
        else
          resumeSweep();
      }
    }
  }
  else
  {
    // failure logic
    info(res->text, true);
    if(full_coverage::ExecuteGoal::DOCK == res->type)
    {
      revertStopStatus();
    }
    else if(full_coverage::ExecuteGoal::UNDOCK == res->type)
    {
       // todo: some failure procedure to move back onto the dock
       changeStatus(STATUS_DOCKED); 
       undockExec = false;
    }
    else if(STATUS_EXECUTING == status)
    {
      info("Failed previous execution, generating new plan.", true);
      if(!generatePlan(getPose()))
      {
        info("Failed to generate new plan after execution.", true);
        endExecution();
      }
    }
    else
      changeStatus(STATUS_IDLE);
  }
}

// Pause robot, reset octomap, resume robot
//void QNode::resetOctomap()
//{
//  clearRobot(false, STATUS_RESET_OCT);
//
//  std_srvs::Empty omsg;
//  oresetClient.call(omsg);
//
//  clearRobot(true);
//  resumeSweep();
//}

// Convenience function for ROS_INFO/ERROR
void QNode::info(std::string text, bool error)
{
  if(!error)
  {
    ROS_INFO_STREAM("Sweep GUI: " + text);
  }
  else
  {
    ROS_ERROR_STREAM("Sweep GUI: " + text);
  }
}

// Called by main window when start button is hit
bool QNode::startPlan(full_coverage::PlanInfo pl)
{
  if(changeStatus(STATUS_EXECUTING))
  {
    full_coverage::ExecuteGoal goal;
    goal.type = full_coverage::ExecuteGoal::START;
    goal.plan = pl;
    info("Sending start command",false);
    sendExecGoal(goal);
    return true;
  }
  else
    return false;
}

// convenience function for sending a goal the the executeClient
void QNode::sendExecGoal(full_coverage::ExecuteGoal goal)
{
  executeClient.sendGoal(goal, boost::bind(&QNode::executeDoneCallback, this, _1, _2), 
                               boost::bind(&QNode::executeActiveCallback, this),
                               boost::bind(&QNode::executeFeedbackCallback, this, _1));
}

// active callback for action lib goals
void QNode::executeActiveCallback()
{
  info("New execute goal active.", false);
}

void QNode::executeFeedbackCallback(const full_coverage::ExecuteFeedbackConstPtr& feedback)
{
  // Visit poses that we have driven to
  full_coverage::Visit srv;
  if(feedback->visited_poses.size() > 0)
    srv.request.visit_poses = feedback->visited_poses;
  else
    srv.request.visit_poses.clear();
  if(feedback->attempted_poses.size() > 0)
    srv.request.attempted_poses = feedback->attempted_poses;
  else
    srv.request.attempted_poses.clear();
  visitClient.call(srv);
}

// resume sweep if in pause state
bool QNode::resumeSweep()
{
  if(STATUS_PAUSED == status)
  {
    if(STATUS_EXECUTING == preStopStatus)
    {
      if(changeStatus(STATUS_EXECUTING))
      {
        full_coverage::ExecuteGoal goal;
        goal.type = full_coverage::ExecuteGoal::RESUME;
        goal.plan.poses.clear();
        info("Sending resume command",false);
        sendExecGoal(goal);
      }
      else
      {
        info("Could not resume motion.",true);
        endExecution();
        return false;
      }
    }
    else if(STATUS_PLANNING == preStopStatus)
    {
      if(!generatePlan(getPose()))
      {
        info("Failed to resume planning after pause.", true);
        endExecution();
      }
    }
    else
    {
      info("We should not have ended up at pause if we were not executing or planning.",true);
      return false;
    }
    return true;
  }
  else
  {
    info("Can't resume if we aren't paused", true);
    return false;
  }
}

// includes all logic needed to cancel all actions and revert to idle
void QNode::endExecution()
{
  planClient.cancelAllGoals();
  executeClient.cancelAllGoals();
  sendStopRobot();
  bool changed = false;
  if(STATUS_IDLE != status)
    changed = true;
  status = STATUS_IDLE;
  preStopStatus = STATUS_IDLE;
  if(changed)
    Q_EMIT newStatus();
}

// stops or clears robot in case of any possible stop action
bool QNode::clearRobot(bool clear, Status unclearedStatus)
{
  if(clear && (STATUS_UNCLEARED == status || STATUS_ESTOPPED == status || STATUS_JOY == status))
  {
    if(isDocked)
    {
      changeStatus(STATUS_DOCKED);
    }
    else
      revertStopStatus();
  }
  else if(!clear)
  {
    saveStopStatus();
    changeStatus(unclearedStatus);
    sendStopRobot();
  }
  else
  {
    info("Robot cleared when it was not uncleared.", true);
    return false;
  }
  return true;
}

// convenience function to send a stop command to the execute server
void QNode::sendStopRobot()
{
  full_coverage::ExecuteGoal goal;
  goal.type = full_coverage::ExecuteGoal::STOP;
  goal.plan.poses.clear();
  info("Sending stop command.", false);
  sendExecGoal(goal);
}

// saves what the status was before we entered any stop state
void QNode::saveStopStatus()
{
  if(status != STATUS_UNCLEARED && status != STATUS_PAUSED && status != STATUS_JOY && status != STATUS_ESTOPPED)
    preStopStatus = status;
}

// uses the preStopStatus to know what state to change to after a stop state
void QNode::revertStopStatus()
{
  if(preStopStatus == STATUS_EXECUTING || preStopStatus == STATUS_PLANNING)
    changeStatus(STATUS_PAUSED, true);
  else
    changeStatus(STATUS_IDLE, true);
}

Status QNode::getPreStopStatus()
{
  return preStopStatus;
}

// Dock robot
bool QNode::dock()
{
  // stop robot and save revert status
  clearRobot(false, STATUS_IDLE);

  // change status to docking
  if(changeStatus(STATUS_DOCKING))
  {
    // call docking algorithm
    full_coverage::ExecuteGoal eGoal;
    eGoal.type = full_coverage::ExecuteGoal::DOCK;
    eGoal.plan.poses.clear();
    info("Sending dock command", false);
    sendExecGoal(eGoal);
    return true;
  }
  else
    return false;
}

bool QNode::undock()
{
  // change status to undocking
  if(changeStatus(STATUS_UNDOCKING))
  {
    // call docking algorithm
    full_coverage::ExecuteGoal eGoal;
    eGoal.type = full_coverage::ExecuteGoal::UNDOCK;
    eGoal.plan.poses.clear();
    info("Sending undock command", false);
    sendExecGoal(eGoal);
    return true;
  }
  else
    return false;
}

bool QNode::undockThenExec()
{
  bool success = undock();
  if(success)
    undockExec = true;
  return success;
}

// start of execution procedure
bool QNode::execute(geometry_msgs::Pose pose)
{
  if(changeStatus(STATUS_EXECUTING))
  {
    // Store start pose for planner
    startPose = pose;

    // turn in place for exploration before generating plan
    full_coverage::ExecuteGoal eGoal;
    eGoal.type = full_coverage::ExecuteGoal::TURN;
    eGoal.plan.poses.clear();
    info("Sending turn in place command.", false);
    sendExecGoal(eGoal);
    return true;
  }
  else
    return false;
}

// start of planning procedure
bool QNode::generatePlan(geometry_msgs::Pose pose)
{
  if(changeStatus(STATUS_PLANNING))
  {
    full_coverage::PlanGoal goal;
    goal.start_pose.header.stamp = ros::Time::now();
    goal.start_pose.header.frame_id = mFrame;
    goal.start_pose.pose = pose;
    goal.second_attempt = secondAttempt;
    planClient.sendGoal(goal, boost::bind(&QNode::planDoneCallback, this, _1, _2));
    return true;
  }
  else
    return false;
}

// Clears the stored wall planner information (where it has visited)
void QNode::eraseMemory()
{
  std_srvs::Empty srv;
  eraseMemoryClient.call(srv);
}

void QNode::clearGmapping()
{
  //std_srvs::Empty omsg;
  //oresetClient.call(omsg);
  std_msgs::Int32 msg;
  msg.data = 1;
  gresetPub.publish(msg);
  eraseMemory();
}

// convenience function to convert pose to map frame
bool QNode::toMapFrame(geometry_msgs::PoseStamped &pose)
{
  if(pose.header.frame_id.compare(mFrame) != 0)
  {
    tf::Stamped<tf::Pose> in, out;
    pose.header.stamp = ros::Time(0);
    tf::poseStampedMsgToTF(pose, in);
    try
    {
      if(listener.waitForTransform(mFrame, pose.header.frame_id, ros::Time(0), ros::Duration(1.0)))
      {
        listener.transformPose(mFrame, in, out);
        tf::poseStampedTFToMsg(out, pose);
        return true;
      }
      else
      {
        std::cout << "wait for transform timed out. mFrame: " << mFrame << " frame_id: " << pose.header.frame_id << std::endl;
        return false;
      }
    }
    catch(...)
    {
      return false;
    }
  }
  else
  {
    // Do nothing, the pose is already in map frame
    return true;
  }
}

}  // namespace sweep_gui
