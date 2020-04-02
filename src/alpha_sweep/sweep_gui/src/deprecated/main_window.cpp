/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/sweep_gui/main_window.hpp"
#include "yaml-cpp/yaml.h"
#include <fstream>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace sweep_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
  , qnode(argc,argv)
  , mapResolution(0.0)
  , mapOriginX(0.0)
  , mapOriginY(0.0)
  , mapWidth(0.0)
  , mapHeight(0.0)
  , robotWidth(0.0)
  , robotHeight(0.0)
  , retryAvailable(false)
  , autoSweep(false)
  , targetDrawn(false)
  , robotDrawn(false)
  , planDrawn(false)
  , planArrowsDrawn(false)
  , status(STATUS_INITIALIZING)
  , me(new MapEditor)
{
  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

  // initialize qnode
  if(!qnode.init())
  {
    QMessageBox msgBox;
    msgBox.setText("QNode not initialized correctly. Exiting program.");
    msgBox.exec();
    close();
    return;
  }
  qnode.getRobotInfo(robotWidth,robotHeight);

	setWindowIcon(QIcon(":/images/icon.png"));
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  QObject::connect(&qnode, SIGNAL(newPlan(bool)), this, SLOT(showPlan(bool)));
  QObject::connect(&qnode, SIGNAL(newMap()), this, SLOT(updateMap()));
  QObject::connect(ui.graphicsView, SIGNAL(mouse(int,int)), this, SLOT(updateCoords(int,int)));
  QObject::connect(&qnode, SIGNAL(updatePose()), this, SLOT(newPose()));
  QObject::connect(&qnode, SIGNAL(updateTarget()), this, SLOT(newTarget()));
  QObject::connect(&qnode, SIGNAL(newStatus()), this, SLOT(receiveStatus()));
  QObject::connect(ui.graphicsView, SIGNAL(planClick(QPointF)), this, SLOT(addPlan(QPointF)));
  QObject::connect(ui.graphicsView, SIGNAL(poseClick(QPointF, QPointF)), this, SLOT(addPose(QPointF, QPointF)));
  QObject::connect(ui.graphicsView, SIGNAL(removeBox(QPointF, QPointF)), this, SLOT(removePoses(QPointF,QPointF)));
  QObject::connect(me, SIGNAL(newScene(QPixmap)), ui.graphicsView, SLOT(setScene(QPixmap)));
  QObject::connect(&qnode, SIGNAL(notCleared()), this, SLOT(showUnclearedMessage()));

  // initial ui stated
  ui.startButton->setEnabled(false);

  // display the map
  updateMap();

  // Set the zoom around robot
  QRectF zoom;
  geometry_msgs::Pose pose = qnode.getPose();
  zoom.setRect(pose.position.x,pose.position.y,10.0*robotWidth,10.0*robotWidth);
  ui.graphicsView->setZoom(zoom);

  status = qnode.getStatus();
}

MainWindow::~MainWindow() {
  if(me)
    delete me;
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/
void MainWindow::addPlan(QPointF p)
{
  geometry_msgs::Pose pose;
  pose.position.x = p.x();
  pose.position.y = p.y();
  pose.position.z = 0;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;

  if(qnode.generatePlan(pose))
    retryAvailable = true;
}

void MainWindow::addPose(QPointF p1, QPointF p2)
{
  geometry_msgs::Pose pose;
  pose.position.x = p1.x();
  pose.position.y = p1.y();
  pose.position.z = 0;

  // drag horizontal
  if(fabs(p1.x()-p2.x()) > fabs(p1.y()-p2.y()))
  {
    if(p1.x()<=p2.x()) // drag right
    {
      pose.orientation.x = 0;
      pose.orientation.y = 0;
      pose.orientation.z = 0;
      pose.orientation.w = 1;
    }
    else // drag left
    {
      pose.orientation.x = 0;
      pose.orientation.y = 0;
      pose.orientation.z = 1;
      pose.orientation.w = 0;
    }
  }
  else
  {
    if(p1.y()<=p2.y()) // drag down
    {
      pose.orientation.x = 0;
      pose.orientation.y = 0;
      pose.orientation.z = 0.7071;
      pose.orientation.w = 0.7071;
    }
    else // drag up
    {
      pose.orientation.x = 0;
      pose.orientation.y = 0;
      pose.orientation.z = -0.7071;
      pose.orientation.w = 0.7071;
    }
  }
  plan.push_back(pose);
  showPlan(false);

  // Enable executing after a plan has been added
  if(status == STATUS_IDLE)
  {
    ui.startButton->setText("Start Sweep");
    ui.startButton->setEnabled(true);
    ui.statusDisplay->setText("Ready to execute");
  }
}

void MainWindow::removePoses(QPointF p1, QPointF p2)
{
  QRectF bounds(p1,p2);
  std::vector<geometry_msgs::Pose> remaining;
  for(int i = 0; i < plan.size(); i++)
  {
    QPointF tempP(plan[i].position.x, plan[i].position.y);
    if(!bounds.contains(tempP))
    {
      remaining.push_back(plan[i]);
    }
  }
  plan.swap(remaining);
  showPlan(false);
}

/*
 * Update target
 */
void MainWindow::newTarget()
{
  // get the current robot pose
  geometry_msgs::Pose pose = qnode.getTarget();

  // Remove last target arrow if it exists
  if(targetDrawn)
    scene->removeItem(targetArrow);

  // Draw target arrow
  targetArrow = scene->addPath(makeArrow(pose, robotWidth/3.0/*mapResolution*/), QPen(QColor(Qt::green)));

  if(robotDrawn)
    robotPathItem->stackBefore(targetArrow);

  targetDrawn = true;
}

/*
 * Update pose
 */
void MainWindow::newPose()
{
  // get the current robot pose
  geometry_msgs::Pose pose = qnode.getPose();

  // calculate angle
  double q0 = pose.orientation.w;
  double q1 = pose.orientation.x;
  double q2 = pose.orientation.y;
  double q3 = pose.orientation.z;
  double theta = atan2(2*(q0*q3+q1*q2),1-2*(pow(q2,2)+pow(q3,2)))*180/M_PI - 90;

  if(robotDrawn)
  {
    // Destroy paths from last pass
    scene->removeItem(vPathItem);
    scene->removeItem(robotPathItem);
    robotDrawn = false;
  }

  // Create scalar transform
  QTransform transformS;
  transformS.scale(1/mapResolution, 1/mapResolution);
  QTransform transformTO;
  transformTO.translate(pose.position.x,
                        pose.position.y);
  transformTO.rotate(theta);

  // Draw sensor path
  QPainterPath sensorPath;
  QRectF sensorRect(-robotWidth/2,
                    -robotHeight/2,
                    robotWidth,
                    robotHeight/4);
  sensorPath.addRect(sensorRect);
  //sensorPath = transformS.map(sensorPath);
  sensorPath = transformTO.map(sensorPath);

  // Add last pose to path
  vPath = vPath.united(sensorPath);

  vPathItem = scene->addPath(vPath, QPen(QColor(0,0,0,0)),QBrush(QColor(Qt::green)));
  vPathItem->setOpacity(0.3);

  // Create graphical robot object
  if(!robotDrawn)
  {
    robotPath = QPainterPath();
    double rectHeight = 2*sqrt(pow(robotHeight/2,2) - pow(robotWidth/2,2));
    QRectF smallRect(-robotWidth/2,-rectHeight/2,robotWidth,rectHeight);
    QRectF largeRect(-robotWidth*2,-rectHeight/2,robotWidth*4,rectHeight);
    QRectF circBound(-robotHeight/2,-robotHeight/2,robotHeight, robotHeight);
    robotPath.addEllipse(circBound);
    QPainterPath temp1;
    temp1.addRect(largeRect);
    robotPath = robotPath.subtracted(temp1);
    QPainterPath temp2;
    temp2.addRect(smallRect);
    robotPath = robotPath.united(temp2);
    //robotPath = transformS.map(robotPath);
  }

  // Draw robot
  robotPathItem = scene->addPath(robotPath,
                                 QPen(QColor(0,0,0,0)),
                                 QBrush(QColor(Qt::red)));
  robotPathItem->setPos(pose.position.x,
                        pose.position.y);
  robotPathItem->setRotation(theta);

  if(targetDrawn)
    robotPathItem->stackBefore(targetArrow);

  robotDrawn = true;
}

/*
 * Update Coordinates
 */
void MainWindow::updateCoords(int x, int y)
{
  double tempX = x;
  double tempY = y;
  std::stringstream ss;
  ss << tempX;
  ss << ", ";
  ss << tempY;
  ui.coordDisplay->setText(QString(ss.str().c_str()));
}

/*
 * Update map
 */
void MainWindow::updateMap()
{
  // Wait to load real map
  int w = 0;
  int h = 0;
  geometry_msgs::Pose mapPose;
  std::vector<int8_t> m;
  if(qnode.getMap(w,h,mapPose,mapResolution,m))
  {
    mapOriginX = mapPose.position.x;
    mapOriginY = mapPose.position.y;
    mapWidth = w * mapResolution;
    mapHeight = h * mapResolution;
    ui.graphicsView->setScene(m, w, h, mapPose, mapResolution);
    scene = ui.graphicsView->scene();
  }
  else
  {
    qnode.info("Failed to load map.",true);
  }
}

void MainWindow::showPlan(bool addToPlan)
{
  // If we generaged new plan data, add it to the plan
  std::vector<geometry_msgs::Pose> tempPlan;
  if(addToPlan)
  {
    qnode.getPlan(tempPlan);
    for(int i = 0; i<tempPlan.size(); i++)
    {
      plan.push_back(tempPlan[i]);
    }
  }

  // Clear previous data
  hidePlan();
  //on_clearButton_released();

  // Display plan
  for(int i = 0; i<plan.size(); i++)
  {
    if(i+1<plan.size())
    {
      planLines.push_back(scene->addLine(plan[i].position.x,
                                         plan[i].position.y,
                                         plan[i+1].position.x,
                                         plan[i+1].position.y));
      planDrawn = true;
    }
    planArrows.push_back(scene->addPath(makeArrow(plan[i], robotWidth/5.0)));//mapResolution)));
    planArrowsDrawn = true;
  }

  ui.showPlanButton->setText("Hide Plan");
}

QPainterPath MainWindow::makeArrow(geometry_msgs::Pose pose, double length)
{
  double q0 = pose.orientation.w;
  double q1 = pose.orientation.x;
  double q2 = pose.orientation.y;
  double q3 = pose.orientation.z;
  double theta = atan2(2*(q0*q3+q1*q2),1-2*(pow(q2,2)+pow(q3,2)))*180/M_PI -90;

  QTransform transformTO;
  transformTO.translate(pose.position.x,
                        pose.position.y);
  transformTO.rotate(theta);

  double halfL = length/2.0;
  QPainterPath arrow(QPointF(0,-length));
  arrow.lineTo(0,0);
  arrow.lineTo(halfL/sqrt(2),0-halfL/sqrt(2));
  arrow.lineTo(0,0);
  arrow.lineTo(-halfL/sqrt(2),0-halfL/sqrt(2));

  arrow = transformTO.map(arrow);
  return arrow;
}

void MainWindow::receiveStatus()
{
  changeStatus(qnode.getStatus());
}

void MainWindow::changeStatus(Status stat)
{
  if(status != stat)
  {
    status = stat;
    switch(status)
    {
      case STATUS_INITIALIZING:
        // fall through
      case STATUS_IDLE:
        ui.statusDisplay->setText("Ready to plan");
        break;
      case STATUS_MAP_SWEEP:
        ui.startButton->setEnabled(false);
        ui.startButton->setText("Executing..");
        ui.statusDisplay->setText("Executing coverage");
        break;
      case STATUS_PLANNING:
        ui.statusDisplay->setText("Planning");
        break;
      case STATUS_READY:
        ui.startButton->setText("Start Sweep");
        ui.startButton->setEnabled(true);
        ui.statusDisplay->setText("Ready to execute");
        break;
      case STATUS_EXECUTING:
        ui.startButton->setEnabled(false);
        ui.startButton->setText("Executing..");
        ui.statusDisplay->setText("Executing");
        break;
      case STATUS_PAUSED:
        ui.startButton->setText("Resume sweep");
        ui.startButton->setEnabled(true);
        ui.statusDisplay->setText("Robot motion paused");
        break;
      case STATUS_UNCLEARED:
        ui.startButton->setText("Clear Robot");
        ui.startButton->setEnabled(true);
        ui.statusDisplay->setText("Robot not cleared for motion");
        break;
      default:
        ui.statusDisplay->setText("Invalid");
        break;
    }
  }
}

/*
 * Command buttons released
 */
void MainWindow::on_cancelButton_released()
{
  qnode.cancelSweep();
}

void MainWindow::on_startButton_released()
{
  // If we are uncleard, start button is changed to clear robot button
  if(STATUS_UNCLEARED == status)
    qnode.clearRobot(true);
  // If we are paused (stopped and then cleared), start button is changed to resume button
  else if(STATUS_PAUSED == status)
    qnode.resumeSweep();
  // otherwise, normal start
  else
  {
    if(plan.size()>0)
    {
      qnode.startPlan(plan, retryAvailable);
    }
    else
    {
      QMessageBox msgBox;
      msgBox.setText("Cannot start sweep until plan is generated or loaded.");
      msgBox.exec();
    }
  }
}

void MainWindow::on_clearPlanButton_released()
{
  plan.clear();
  hidePlan();
  qnode.planCleared();
}

void MainWindow::hidePlan()
{
  ui.showPlanButton->setText("Show Plan");

  // Clear plan lines
  if(planDrawn)
  {
    for(int i = 0; i < planLines.size(); i++)
    {
      scene->removeItem(planLines[i]);
    }
    planDrawn = false;
  }

  // Clear plan arrows
  if(planArrowsDrawn)
  {
    for(int i = 0; i < planArrows.size(); i++)
    {
      scene->removeItem(planArrows[i]);
    }
    planArrowsDrawn = false;
  }
}

void MainWindow::on_showPlanButton_released()
{
  if(planDrawn)
  {
    hidePlan();
  }
  else
  {
    showPlan(false);
  }
}

void MainWindow::on_panTool_released()
{
  ui.graphicsView->setMouseState(PAN);
}

void MainWindow::on_planFromPosTool_released()
{
  qnode.generatePlan(qnode.getPose());
}

void MainWindow::on_planFromClickTool_released()
{
  ui.graphicsView->setMouseState(GEN_PLAN);
}

void MainWindow::on_addFromClickTool_released()
{
  ui.graphicsView->setMouseState(ADD_POSE);
}

void MainWindow::on_removePosesTool_released()
{
  ui.graphicsView->setMouseState(REM_POSES);
}

void MainWindow::on_clearButton_released()
{
  // Clear plan lines
  hidePlan();

  // Clear robot and path
  if(robotDrawn)
  {
    // Destroy paths from last pass
    vPath = QPainterPath();
    scene->removeItem(vPathItem);
    scene->removeItem(robotPathItem);
    robotDrawn = false;
  }

  if(targetDrawn)
  {
    scene->removeItem(targetArrow);
    targetDrawn = false;
  }
}

void MainWindow::on_stopButton_released()
{
  qnode.clearRobot(false);
}

void MainWindow::on_mapSweepButton_released()
{
  qnode.mapSweep(qnode.getPose());
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/


/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_action_Preferences_triggered()
{
  // Do nothing
}

void MainWindow::on_action_Save_triggered()
{
  QString fileName = QFileDialog::getSaveFileName(this, tr("Save Plan"), "~/Documents/", tr("Plan Files (*.plan)"));
  YAML::Node saveNode;

  if(!fileName.isNull())
  {
    for(int i = 0; i < plan.size(); i++)
    {
      YAML::Node tempPose;
      YAML::Node pos;
      pos["x"] = plan[i].position.x;
      pos["y"] = plan[i].position.y;
      pos["z"] = plan[i].position.z;
      YAML::Node orientation;
      orientation["x"] = plan[i].orientation.x;
      orientation["y"] = plan[i].orientation.y;
      orientation["z"] = plan[i].orientation.z;
      orientation["w"] = plan[i].orientation.w;
      tempPose["position"] = pos;
      tempPose["orientation"] = orientation;
      saveNode.push_back(tempPose);
    }

    std::string tempName = fileName.toStdString();
    if(tempName.find(".") == std::string::npos)
      tempName.append(".plan");
    std::ofstream fout(tempName.c_str());
    fout << saveNode;
    fout.close();
  }
}

void MainWindow::on_action_Load_triggered()
{
  QString fileName = QFileDialog::getOpenFileName(this, tr("Load Plan"), "~/Documents", tr("Plan Files (*.plan)"));

  if(!fileName.isNull())
  {
    try
    {
      YAML::Node nodes = YAML::LoadFile(fileName.toStdString());
      plan.clear();

      for (std::size_t i=0;i<nodes.size();i++)
      {
        YAML::Node node = nodes[i];
        geometry_msgs::Pose pose;
        pose.position.x = node["position"]["x"].as<double>();
        pose.position.y = node["position"]["y"].as<double>();
        pose.position.z = node["position"]["z"].as<double>();
        pose.orientation.x = node["orientation"]["x"].as<double>();
        pose.orientation.y = node["orientation"]["y"].as<double>();
        pose.orientation.z = node["orientation"]["z"].as<double>();
        pose.orientation.w = node["orientation"]["w"].as<double>();

        plan.push_back(pose);
      }
    }
    catch(...)
    {
      QMessageBox msgBox;
      msgBox.setText("Could Load Plan. Plan file is ill-formed.");
      msgBox.exec();
    }

    // Enable executing after a plan has been added
    if((status == STATUS_IDLE || status == STATUS_READY) && plan.size() > 0)
    {
      ui.startButton->setText("Start Sweep");
      ui.startButton->setEnabled(true);
      ui.statusDisplay->setText("Ready to execute");
      showPlan(false);
    }
  }
}

void MainWindow::on_action_Editor_triggered()
{
  // Create map editor if it has not yet been created
  if(!me)
    me = new MapEditor(this);

  // Set the scene of the map editor
  me->setScene(ui.graphicsView->getPixMap());

  // Show map editor and raise it to the front if it is hiding
  me->show();
  me->raise();
}

void MainWindow::showUnclearedMessage()
{
  QMessageBox msgBox;
  msgBox.setText("Cannot start perform action until robot is cleared.");
  msgBox.exec();
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::closeEvent(QCloseEvent *event)
{
  // Add things we need to do on close here

	QMainWindow::closeEvent(event);
}

}  // namespace sweep_gui

