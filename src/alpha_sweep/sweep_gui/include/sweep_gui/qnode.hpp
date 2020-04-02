/**
 * @file /include/sweep_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef sweep_gui_QNODE_HPP_
#define sweep_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <full_coverage/PlanAction.h>
#include <full_coverage/ExecuteAction.h>
#include <string>
#include <vector>
#include <QStringListModel>
#include <QCoreApplication>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <full_coverage/BoolReq.h>
#include <full_coverage/SweepProgress.h>
#include <full_coverage/GetNavPlan.h>
#include <full_coverage/MakePlan.h>
#include <full_coverage/ExecutePlan.h>
#include <full_coverage/Visit.h>
#include <full_coverage/FollowStatus.h>
#include <full_coverage/PlanInfo.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include "../include/sweep_gui/sweep_gui_enums.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <full_coverage/DebugGrid.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace sweep_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QObject {
  Q_OBJECT
public:
  QNode();
  virtual ~QNode();
  bool init();
  void info(std::string text, bool error);

  bool getMap(int &w, int &h, geometry_msgs::Pose &pose, double &res, std::vector<int8_t> &data);
  void getRobotInfo(double &rw, double &rh, double &sw, double &sl, double &sx, double &sy);
  bool getPlan(std::vector<geometry_msgs::Pose> &pl);
  geometry_msgs::Pose getPose();
  geometry_msgs::Pose getDockPose();
  geometry_msgs::Pose getTarget();
  geometry_msgs::Pose getNavPose();
  Status getStatus();
  full_coverage::DebugGrid getDebugGrid();
  Status getPreStopStatus();

  // Commands to execute
  bool startPlan(full_coverage::PlanInfo pl);
  bool generatePlan(geometry_msgs::Pose pose);
  bool clearRobot(bool clear, Status unclearedStatus = STATUS_UNCLEARED);
  bool resumeSweep();
  bool execute(geometry_msgs::Pose pose);
  void enableJoy(bool enable);
  void endExecution();
  void eraseMemory();
  void clearGmapping();
  void sendShutdown();
  bool dock();
  bool undock();
  bool undockThenExec();

  // frame transformer
  bool toMapFrame(geometry_msgs::PoseStamped &pose);

Q_SIGNALS:
  void rosShutdown();
  void newPlan();
  void updatePose();
  void updateTarget();
  void updateNavPose();
  void newStatus();
  void newMap();
  void notCleared();
  void inJoy();
  void eStopped();
  void execDone();
  void newDebugMsg();
  void newSafeD(bool);
  void newSafeT(bool);
  void saveRequest(QString);

private:
  bool publishReady;
  ros::NodeHandle n;
  ros::AsyncSpinner spinner;
  tf::TransformListener listener;
  actionlib::SimpleActionClient<full_coverage::PlanAction> planClient;
  actionlib::SimpleActionClient<full_coverage::ExecuteAction> executeClient;
  ros::ServiceClient visitClient;
  void planDoneCallback(const actionlib::SimpleClientGoalState& state, const full_coverage::PlanResultConstPtr& res);
  void executeDoneCallback(const actionlib::SimpleClientGoalState& state, const full_coverage::ExecuteResultConstPtr& res);
  void executeActiveCallback();
  void executeFeedbackCallback(const full_coverage::ExecuteFeedbackConstPtr& res);
  void sendExecGoal(full_coverage::ExecuteGoal goal);
  void sendStopRobot();
  full_coverage::PlanInfo plan;
  geometry_msgs::Pose startPose;
  geometry_msgs::PoseStamped dockPose;
  bool turnSuccess;
  bool firstPose;
  void initRosPubs();
  bool undockExec;
  bool secondAttempt;
  bool isDocked;
  bool isEstopped;

  // Robot data
  double robotWidth;
  double robotLength;
  double sensorW;
  double sensorL;
  double sensorX;
  double sensorY;

  // dock subscriber
  ros::Subscriber dockSub;
  void dockCallback(const std_msgs::Int8 msg);

  // Joystick publisher
  ros::Publisher joyEnablePub;

  // E-stop sub
  ros::Subscriber estopSub;
  void estopCallback(const std_msgs::Bool pressed);

  // safe drive subscriber
  ros::Subscriber sdSub;
  ros::Subscriber stSub;
  bool safeDrive;
  bool safeTurn;
  void sdCallback(const std_msgs::Bool safe);
  void stCallback(const std_msgs::Bool safe);

  // 3d mapping ready sub
  ros::Subscriber mappingReadySub;
  void mappingReadyCallback(const std_msgs::Bool msg);
  bool mappingReady;

  // Occupancy map subscriber and callback
  double mRes;
  int mWidth;
  int mHeight;
  geometry_msgs::Pose mPose;
  std::vector<int8_t> map;
  bool useCostmap;
  std::string mapTopic;
  bool gotMap;
  std::string mFrame;
  ros::Subscriber mapClient;
  void mapCallback(nav_msgs::OccupancyGrid grid);
  ros::Subscriber mapUpdatesClient;
  void mapUpdatesCallback(const map_msgs::OccupancyGridUpdate::ConstPtr &msg);
  void getLethal();
  ros::Time lastMap;
  bool debug;
  ros::Subscriber debugGridSub;
  void debugCallback(full_coverage::DebugGrid msg);
  full_coverage::DebugGrid debugMsg;

  // Status/state tracking
  Status status;
  Status preStopStatus;
  bool changeStatus(Status stat, bool allowClear = false);
  void saveStopStatus();
  void revertStopStatus();

  // pose tracking
  ros::Subscriber poseSub;
  void poseCallback(geometry_msgs::PoseStamped pose);
  geometry_msgs::PoseStamped robotPose;
  ros::Time lastSent;
  double getDist(geometry_msgs::PoseStamped &p1, geometry_msgs::PoseStamped &p2);

  // target pose tracking
  ros::Subscriber targetSub;
  void targetCallback(geometry_msgs::PoseStamped pose);
  geometry_msgs::PoseStamped targetPose;

  // move base nav pose tracking
  ros::Subscriber navPoseSub;
  void navPoseCallback(geometry_msgs::PoseStamped pose);
  geometry_msgs::PoseStamped navPose;

  // erase path memory service client
  ros::ServiceClient eraseMemoryClient;

  // gmapping and octomap resets
  ros::ServiceClient oresetClient;
  geometry_msgs::PoseStamped lastResetPose;
  ros::Time lastResetTime;
  void resetOctomap();
  ros::Publisher gresetPub;
  ros::Subscriber gresetSub;
  void gresetCallback(std_msgs::Int32 msg);

  // map saving
  int saveCount;

  // shutdown publisher
  ros::Publisher shutdownPub;

  // For Wheel Lights
  //ros::Publisher wheelLightsPub;
  //void setWheelLights();

};

}  // namespace sweep_gui

#endif /* sweep_gui_QNODE_HPP_ */
