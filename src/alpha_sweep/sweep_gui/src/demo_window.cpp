/*********************************************************************
*
*  © (or copyright) 2020. Triad National Security, LLC.
*  All rights reserved.
*  This program was produced under U.S. Government contract 
*  89233218CNA000001 for Los AlamosNational Laboratory (LANL), 
*  which is operated by Triad National Security, LLC for the U.S.
*  Department of Energy/National Nuclear Security Administration. 
*  All rights in the program are reserved by Triad National 
*  Security, LLC, and the U.S. Department of Energy/National Nuclear
*  Security Administration. The Government is granted for itself 
*  and others acting on its behalf a nonexclusive, paid-up, 
*  irrevocable worldwide license in this material to reproduce, 
*  prepare derivative works, distribute copies to the public, 
*  perform publicly and display publicly, and to permit others 
*  to do so.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Alex von Sternberg
*
* Description: Implementation of GUI for alpha sweep demo
*********************************************************************/


#include <QtWidgets>
#include <QMessageBox>
#include <iostream>
#include "../include/sweep_gui/demo_window.hpp"
#include <fstream>

namespace sweep_gui {

using namespace Qt;

DemoWindow::DemoWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent)
  , mapResolution(0.0)
  , mapOriginX(0.0)
  , mapOriginY(0.0)
  , mapWidth(0.0)
  , mapHeight(0.0)
  , robotWidth(0.0)
  , robotHeight(0.0)
  , scene(0)
  , vPathItem(0)
  , robotPathItem(0)
  , targetArrow(0)
  , navArrow(0)
  , dockArrow(0)
  , robotDrawn(false)
  , centerOnRob(true)
  , lastTheta(0.0)
  , status(STATUS_INITIALIZING)
  , init(false)
  , hasMap(false)
  , sensorW(0.0)
  , sensorL(0.0)
  , sensorX(0.0)
  , sensorY(0.0)
{
  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

  // create and initialize qnode
  qnode = new QNode();
  if(!qnode->init())
  {
    QMessageBox msgBox;
    msgBox.setText("QNode not initialized correctly. Exiting program.");
    msgBox.exec();
    close();
    return;
  }
  qnode->getRobotInfo(robotWidth,robotHeight,sensorW,sensorL,sensorX,sensorY);

  //adjust sensor info for fact that its the corner of the rectangle
  if(sensorX > 0)
    sensorX = sensorX + sensorW/2.0;
  else
    sensorX = sensorX - sensorW/2.0;
  if(sensorY > 0)
    sensorY = sensorY + sensorL/2.0;
  else
    sensorY = sensorY - sensorL/2.0;

  setWindowIcon(QIcon(":/images/icon.png"));
  QObject::connect(qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  QObject::connect(qnode, SIGNAL(newPlan()), this, SLOT(showPlan()));
  QObject::connect(qnode, SIGNAL(newMap()), this, SLOT(updateMap()));
  QObject::connect(qnode, SIGNAL(updatePose()), this, SLOT(newPose()));
  QObject::connect(qnode, SIGNAL(updateTarget()), this, SLOT(newTarget()));
  QObject::connect(qnode, SIGNAL(updateNavPose()), this, SLOT(newNavPose()));
  QObject::connect(qnode, SIGNAL(newStatus()), this, SLOT(receiveStatus()));
  QObject::connect(qnode, SIGNAL(notCleared()), this, SLOT(showUnclearedMessage()));
  QObject::connect(qnode, SIGNAL(inJoy()), this, SLOT(showJoyMessage()));
  QObject::connect(qnode, SIGNAL(eStopped()), this, SLOT(showEStopMessage()));
  QObject::connect(qnode, SIGNAL(execDone()), this, SLOT(showDoneMessage()));
  QObject::connect(qnode, SIGNAL(newDebugMsg()), this, SLOT(showDebugInfo()));
  QObject::connect(qnode, SIGNAL(newSafeD(bool)), ui.sdWidget, SLOT(setState(bool)));
  QObject::connect(qnode, SIGNAL(newSafeT(bool)), ui.stWidget, SLOT(setState(bool)));
  QObject::connect(qnode, SIGNAL(saveRequest(QString)), this, SLOT(saveMap(QString)));

  // initial ui state
  ui.startButton->setEnabled(false);
  ui.joyButton->setEnabled(false);
  ui.cancelButton->setEnabled(false);
  ui.centerBox->setCheckState(Qt::Checked);
  ui.graphicsView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  ui.graphicsView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

  // display the map. If we don't have a map display a temporary scene.
  if(!updateMap())
  {
    scene = new QGraphicsScene(0,0,100,100,ui.graphicsView);
    QGraphicsTextItem *text = scene->addText("No map available. Check data sources.");
    QTransform trans;
    trans.scale(1,-1);
    text->setTransform(trans);
    text->setPos(50-text->boundingRect().width()/2.0,50-text->boundingRect().height()/2.0);
    ui.graphicsView->QGraphicsView::setScene(scene);
  }

  changeStatus(qnode->getStatus());
  init = true;
}

DemoWindow::~DemoWindow()
{
  delete qnode;
}

bool DemoWindow::isInit()
{
  return hasMap && init;
}

/*****************************************************************************
** Slot Manual Connections
*****************************************************************************/

/*
 * Update target
 */
void DemoWindow::newTarget()
{
  if(!isInit())
    return;

  // get the current robot pose
  geometry_msgs::Pose pose = qnode->getTarget();

  // Remove last target arrow if it exists
  if(targetArrow)
  {
    scene->removeItem(targetArrow);
    delete targetArrow;
    targetArrow = 0;
  }

  // Draw target arrow
  targetArrow = scene->addPath(makeArrow(pose, robotWidth/3.0), QPen(QBrush(QColor(Qt::green)),0));

  if(robotPathItem && targetArrow)
    robotPathItem->stackBefore(targetArrow);
}

/*
 * Update nav pose
 */
void DemoWindow::newNavPose()
{
  if(!isInit())
    return;

  // get the current robot pose
  geometry_msgs::Pose pose = qnode->getNavPose();

  // Remove last nav pose arrow if it exists
  if(navArrow)
  {
    scene->removeItem(navArrow);
    delete navArrow;
    navArrow = 0;
  }

  // Draw nav pose arrow
  navArrow = scene->addPath(makeArrow(pose, robotWidth), QPen(QBrush(QColor(Qt::red)),0));

  if(robotPathItem && navArrow)
    robotPathItem->stackBefore(navArrow);
}

/*
 * Update pose
 */
void DemoWindow::newPose()
{
  if(!isInit())
    return;

  // get the current robot pose
  geometry_msgs::Pose pose = qnode->getPose();

  // calculate angle
  double q0 = pose.orientation.w;
  double q1 = pose.orientation.x;
  double q2 = pose.orientation.y;
  double q3 = pose.orientation.z;
  double theta = atan2(2*(q0*q3+q1*q2),1-2*(pow(q2,2)+pow(q3,2)))*180/M_PI - 90;

  // Destroy paths from last pass
  if(vPathItem)
  {
    scene->removeItem(vPathItem);
    delete vPathItem;
    vPathItem = 0;
  }
  if(robotPathItem)
  {
    scene->removeItem(robotPathItem);
    delete robotPathItem;
    robotPathItem = 0;
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
  QRectF sensorRect(sensorX,
                    sensorY,
                    sensorW,
                    sensorL);
  sensorPath.addRect(sensorRect);
  sensorPath = transformTO.map(sensorPath);

  // Add last pose to path
  vPath = vPath.united(sensorPath);

  vPathItem = scene->addPath(vPath, QPen(QBrush(QColor(0,0,0,0)),0),QBrush(QColor(Qt::green)));
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
    robotDrawn = true;
  }

  // Draw robot
  robotPathItem = scene->addPath(robotPath,
                                 QPen(QBrush(QColor(0,0,0,0)),0),
                                 QBrush(QColor(Qt::red)));
  robotPathItem->setPos(pose.position.x,
                        pose.position.y);
  robotPathItem->setRotation(theta);
  robotPathItem->setZValue(scene->items().first()->zValue()+1);

  // Center screen on robot
  if(centerOnRob)
  {
    ui.graphicsView->rotate(-theta-lastTheta);
    lastTheta = -theta;
    ui.graphicsView->centerOn(QPointF(pose.position.x,pose.position.y));
  }

  // Also update the dock pose
  // get the current dock pose
  //geometry_msgs::Pose dpose = qnode->getDockPose();

  //// Remove last dock arrow if it exists
  //if(dockArrow)
  //{
  //  scene->removeItem(dockArrow);
  //  delete dockArrow;
  //  dockArrow = 0;
  //}

  // Draw dock arrow
  //dockArrow = scene->addPath(makeArrow(dpose, robotWidth), QPen(QBrush(QColor(Qt::blue)), 0));

  //if(robotPathItem && dockArrow)
  //  robotPathItem->stackBefore(dockArrow);
}

/*
 * Update Coordinates
 */
//void DemoWindow::updateCoords(int x, int y)
//{
//  double tempX = x;
//  double tempY = y;
//  std::stringstream ss;
//  ss << tempX;
//  ss << ", ";
//  ss << tempY;
//  ui.coordDisplay->setText(QString(ss.str().c_str()));
//}

/*
 * Update map
 */
bool DemoWindow::updateMap()
{
  // Wait to load real map
  int w = 0;
  int h = 0;
  geometry_msgs::Pose mapPose;
  std::vector<int8_t> m;
  if(qnode->getMap(w,h,mapPose,mapResolution,m))
  {
    if(!hasMap && scene)
      delete scene;
    mapOriginX = mapPose.position.x;
    mapOriginY = mapPose.position.y;
    mapWidth = w * mapResolution;
    mapHeight = h * mapResolution;
    ui.graphicsView->setScene(m, w, h, mapPose, mapResolution);
    scene = ui.graphicsView->scene();

    // If this is the first map load, set the zoom around robot
    if(!hasMap)
    {
      windowHandle()->setScreen(QApplication::screens().last());
      setWindowState(Qt::WindowMaximized);
      hasMap = true;
      on_zoomResetButton_clicked();
    }
    return true;
  }
  else
  {
    qnode->info("Failed to load map.",true);
    return false;
  }
}

/*
 * Show the plan on the display
 */
void DemoWindow::showPlan()
{
  // If we generaged new plan data, add it to the plan
  qnode->getPlan(plan);

  // Clear previous data
  hidePlan();

  // Display plan
  for(int i = 0; i<plan.size(); i++)
  {
    if(i+1<plan.size())
    {
      planLines.push_back(scene->addLine(plan[i].position.x,
                                         plan[i].position.y,
                                         plan[i+1].position.x,
                                         plan[i+1].position.y, QPen(QBrush(QColor(Qt::black)),0)));
    }
    planArrows.push_back(scene->addPath(makeArrow(plan[i], robotWidth/5.0), QPen(QBrush(QColor(Qt::black)),0)));//mapResolution)));
  }
}

void DemoWindow::receiveStatus()
{
  changeStatus(qnode->getStatus());
}

void DemoWindow::showDebugInfo()
{
  for(int i = 0; i<debugPolygons.size(); i++)
  {
    scene->removeItem(debugPolygons[i]);
    delete debugPolygons[i];
  }
  debugPolygons.clear();
  std::vector<QPainterPath> debugPath;
  debugPath.resize(6);

  full_coverage::DebugGrid dm = qnode->getDebugGrid();
  for(int i = 0; i<dm.coarse_cells.size(); i++)
  {
    geometry_msgs::PoseStamped tl;
    tl.pose = dm.coarse_cells[i].tl_vertex;
    tl.header.frame_id = dm.frame_id;
    tl.header.stamp = ros::Time::now();
    qnode->toMapFrame(tl);

    geometry_msgs::PoseStamped tr;
    tr.pose = dm.coarse_cells[i].tr_vertex;
    tr.header.frame_id = dm.frame_id;
    tr.header.stamp = ros::Time::now();
    qnode->toMapFrame(tr);

    geometry_msgs::PoseStamped br;
    br.pose = dm.coarse_cells[i].br_vertex;
    br.header.frame_id = dm.frame_id;
    br.header.stamp = ros::Time::now();
    qnode->toMapFrame(br);

    geometry_msgs::PoseStamped bl;
    bl.pose = dm.coarse_cells[i].bl_vertex;
    bl.header.frame_id = dm.frame_id;
    bl.header.stamp = ros::Time::now();
    qnode->toMapFrame(bl);

    QVector<QPointF> points;
    points.push_back(QPointF(tl.pose.position.x,tl.pose.position.y));
    points.push_back(QPointF(tr.pose.position.x,tr.pose.position.y));
    points.push_back(QPointF(br.pose.position.x,br.pose.position.y));
    points.push_back(QPointF(bl.pose.position.x,bl.pose.position.y));
    QPolygonF poly(points);
    int type = dm.coarse_cells[i].cell_type;
    switch(type)
    {
      case 0: // open cell
        debugPath[0].addPolygon(poly);
        break;
      case 1: // border cell
        debugPath[1].addPolygon(poly);
        break;
      case 2: // obstruction
        debugPath[2].addPolygon(poly);
        break;
      case 10: // visited cell
        debugPath[3].addPolygon(poly);
        break;
      case 11: // attempted cell
        debugPath[4].addPolygon(poly);
        break;
      default: // unknown
        debugPath[5].addPolygon(poly);
        break;
    }
  }

  debugPolygons.push_back(scene->addPath(debugPath[0],QPen(QBrush(QColor(0,0,0,0)),0),QBrush(QColor(Qt::blue))));
  debugPolygons[0]->setOpacity(0.3);
  debugPolygons.push_back(scene->addPath(debugPath[1],QPen(QBrush(QColor(0,0,0,0)),0),QBrush(QColor(Qt::yellow))));
  debugPolygons[1]->setOpacity(0.3);
  debugPolygons.push_back(scene->addPath(debugPath[2],QPen(QBrush(QColor(0,0,0,0)),0),QBrush(QColor(Qt::red))));
  debugPolygons[2]->setOpacity(0.3);
  debugPolygons.push_back(scene->addPath(debugPath[3],QPen(QBrush(QColor(0,0,0,0)),0),QBrush(QColor(Qt::green))));
  debugPolygons[3]->setOpacity(0.3);
  debugPolygons.push_back(scene->addPath(debugPath[4],QPen(QBrush(QColor(0,0,0,0)),0),QBrush(QColor(Qt::cyan))));
  debugPolygons[4]->setOpacity(0.3);
  debugPolygons.push_back(scene->addPath(debugPath[5],QPen(QBrush(QColor(0,0,0,0)),0),QBrush(QColor(Qt::gray))));
  debugPolygons[5]->setOpacity(0.3);
}

void DemoWindow::saveMap(QString fileName)
{
  QRectF rectf = ui.graphicsView->sceneRect();
  QPolygon poly = ui.graphicsView->mapFromScene(rectf);
  if(poly.size() > 1)
  {
    int x,y;
    poly.point(0,&x,&y);
    QPoint tl(x,y);
    QPoint br(x,y);
    for(int i = 1; i < poly.size(); i++)
    {
      poly.point(i,&x,&y);
      if(x < tl.x())
        tl.setX(x);
      if(y < tl.y())
        tl.setY(y);
      if(x > br.x())
        br.setX(x);
      if(y > br.y())
        br.setY(y);
    }
    QRect rect(tl,br);

    QPixmap pMap(rect.width(),rect.height());
    pMap.fill();
    QPainter painter(&pMap);
    ui.graphicsView->scene()->render(&painter);
    painter.end();
    if(pMap.save(fileName))
      std::cout << "Saving file: " << fileName.toStdString() << std::endl;
    else
      std::cout << "Failed to save file: " << fileName.toStdString() << std::endl;
  }
  else
    std::cout << "Failed to save file: " << fileName.toStdString() << std::endl;
}

/*****************************************************************************
** Button Auto Connections
*****************************************************************************/

void DemoWindow::on_quitButton_clicked()
{
  close();
}

void DemoWindow::on_startButton_clicked()
{
  if(!isInit())
  {
    showNotInitMessage();
    return;
  }
  switch(status)
  {
    case STATUS_DOCKED:
      // undock then execute
      qnode->undockThenExec();
      break;
    case STATUS_IDLE:
      // Start normally
      qnode->execute(qnode->getPose());
      break;
    case STATUS_PLANNING:
      // Stop robot/ set uncleared
      // Fall through
    case STATUS_EXECUTING:
      // Stop robot/set uncleared
      qnode->clearRobot(false);
      break;
    case STATUS_PAUSED:
      // Resume Sweep
      qnode->resumeSweep();
      break;
    case STATUS_UNCLEARED:
      // Clear robot
      qnode->clearRobot(true);
      break;
    case STATUS_JOY:
      // Start button disabled. Should not happen.
      // Fall through
    case STATUS_ESTOPPED:
      // Start button disabled. Should not happen.
      // Fall through
    case STATUS_INITIALIZING:
      // Start button disabled. Should not happen.
      // Fall through
    case STATUS_DOCKING:
      // Start button disabled. Should not happen.
      // Fall through
    case STATUS_UNDOCKING:
      // Start button disabled. Should not happen.
      // Fall through
    default:
      QMessageBox msgBox;
      msgBox.setText("Start button was pressed when it should be disabled. Check on_startButton_clicked().");
      msgBox.exec();
      break;
  }
}

void DemoWindow::on_dockButton_clicked()
{
  if(!isInit())
  {
    showNotInitMessage();
    return;
  }

  switch(status)
  {
    case STATUS_DOCKED:
      // Undock
      qnode->undock();
      break;
    case STATUS_PLANNING:
      // fall through to normal dock routine
    case STATUS_EXECUTING:
      // fall through to normal dock routine
    case STATUS_PAUSED:
      // fall through to normal dock routine
    case STATUS_IDLE:
      // Dock robot
      qnode->dock();
      break;
    case STATUS_UNCLEARED:
      // Start button disabled. Should not happen.
      // Fall through
    case STATUS_JOY:
      // Start button disabled. Should not happen.
      // Fall through
    case STATUS_ESTOPPED:
      // Start button disabled. Should not happen.
      // Fall through
    case STATUS_INITIALIZING:
      // Start button disabled. Should not happen.
      // Fall through
    case STATUS_DOCKING:
      // Start button disabled. Should not happen.
      // Fall through
    case STATUS_UNDOCKING:
      // Start button disabled. Should not happen.
      // Fall through
    default:
      QMessageBox msgBox;
      msgBox.setText("Dock button was pressed when it should be disabled. Check on_dockButton_clicked().");
      msgBox.exec();
      break;
  }
}

void DemoWindow::on_joyButton_clicked()
{
  if(STATUS_JOY == status)
    qnode->enableJoy(false);
  else
    qnode->enableJoy(true);
}

void DemoWindow::on_cancelButton_clicked()
{
  if(!isInit())
  {
    showNotInitMessage();
    return;
  }

  if(STATUS_IDLE == status)
  {
    if(vPathItem)
    {
      scene->removeItem(vPathItem);
      delete vPathItem;
      vPathItem = 0;
    }
    vPath = QPainterPath();

    qnode->clearGmapping();
  }
  else
    qnode->endExecution();
  hidePlan();
}

void DemoWindow::on_centerBox_stateChanged(int state)
{
  if(Qt::Unchecked == state)
  {
    centerOnRob = false;
  }
  else if(Qt::Checked == state)
  {
    centerOnRob = true;
  }
}

void DemoWindow::on_zMinusButton_clicked()
{
  if(hasMap)
    ui.graphicsView->scaleBy(3.0/4.0);
}

void DemoWindow::on_zPlusButton_clicked()
{
  if(hasMap)
    ui.graphicsView->scaleBy(4.0/3.0);
}

void DemoWindow::on_zoomResetButton_clicked()
{
  if(hasMap)
  {
    QRectF zoom;
    geometry_msgs::Pose pose = qnode->getPose();
    zoom.setRect(pose.position.x,pose.position.y,20.0*robotWidth,20.0*robotWidth);
    ui.graphicsView->setZoom(zoom);
  }
}

void DemoWindow::hidePlan()
{
  // Clear plan lines
  for(int i = 0; i < planLines.size(); i++)
  {
    scene->removeItem(planLines[i]);
    delete planLines[i];
  }
  planLines.clear();

  // Clear plan arrows
  for(int i = 0; i < planArrows.size(); i++)
  {
    scene->removeItem(planArrows[i]);
    delete planArrows[i];
  }
  planArrows.clear();
}

/*****************************************************************************
** Menu Auto Connections
*****************************************************************************/

void DemoWindow::on_action_Preferences_triggered()
{
  // Do nothing
}

void DemoWindow::showUnclearedMessage()
{
  QMessageBox msgBox;
  msgBox.setText("Cannot perform action until robot is cleared.");
  msgBox.exec();
}

void DemoWindow::showJoyMessage()
{
  QMessageBox msgBox;
  msgBox.setText("Cannot perform action while in joy mode. Disable joystick first.");
  msgBox.exec();
}

void DemoWindow::showEStopMessage()
{
  QMessageBox msgBox;
  msgBox.setText("Cannot perform action while e-stop is pressed.");
  msgBox.exec();
}

void DemoWindow::showDoneMessage()
{
  QMessageBox msgBox;
  msgBox.setText("Sweep execution finished.");
  msgBox.exec();
}

void DemoWindow::showNotInitMessage()
{
  QMessageBox msgBox;
  msgBox.setText("Init was not successful or we have not received a valid map.");
  msgBox.exec();
}
/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void DemoWindow::closeEvent(QCloseEvent *event)
{
  // Add things we need to do on close here
  qnode->sendShutdown();

  QMainWindow::closeEvent(event);
}

/*****************************************************************************
** Helper functions
*****************************************************************************/

QPainterPath DemoWindow::makeArrow(geometry_msgs::Pose pose, double length)
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

void DemoWindow::changeStatus(Status stat)
{
  if(status != stat)
  {
    status = stat;
    switch(status)
    {
      case STATUS_INITIALIZING:
        ui.statusDisplay->setText("Initializing");
        break;
      case STATUS_IDLE:
        ui.dockButton->setText("Dock");
        ui.dockButton->setEnabled(true);
        ui.startButton->setText("Start Sweep");
        ui.startButton->setEnabled(true);
        ui.joyButton->setText("Enable Joystick");
        ui.joyButton->setEnabled(true);
        ui.cancelButton->setText("New Room");
        ui.cancelButton->setEnabled(true);
        ui.statusDisplay->setText("Ready to execute");
        break;
      case STATUS_PLANNING:
        ui.dockButton->setText("Dock");
        ui.dockButton->setEnabled(true);
        ui.startButton->setText("Stop Robot");
        ui.startButton->setEnabled(true);
        ui.joyButton->setText("Enable Joystick");
        ui.joyButton->setEnabled(true);
        ui.cancelButton->setText("Cancel Sweep");
        ui.cancelButton->setEnabled(true);
        ui.statusDisplay->setText("Generating plan");
        break;
      case STATUS_EXECUTING:
        ui.dockButton->setText("Dock");
        ui.dockButton->setEnabled(true);
        ui.startButton->setText("Stop Robot");
        ui.startButton->setEnabled(true);
        ui.joyButton->setText("Enable Joystick");
        ui.joyButton->setEnabled(true);
        ui.cancelButton->setText("Cancel Sweep");
        ui.cancelButton->setEnabled(true);
        ui.statusDisplay->setText("Executing sweep");
        break;
      case STATUS_PAUSED:
        ui.dockButton->setText("Dock");
        ui.dockButton->setEnabled(true);
        ui.startButton->setText("Resume sweep");
        ui.startButton->setEnabled(true);
        ui.joyButton->setText("Enable Joystick");
        ui.joyButton->setEnabled(true);
        ui.cancelButton->setText("Cancel Sweep");
        ui.cancelButton->setEnabled(true);
        ui.statusDisplay->setText("Robot motion paused");
        break;
      case STATUS_UNCLEARED:
        ui.dockButton->setText("Dock");
        ui.dockButton->setEnabled(false);
        ui.startButton->setText("Clear Robot");
        ui.startButton->setEnabled(true);
        ui.joyButton->setText("Enable Joystick");
        ui.joyButton->setEnabled(false);
        ui.cancelButton->setEnabled(false);
        ui.statusDisplay->setText("Robot is not cleared for motion");
        break;
      case STATUS_JOY:
        ui.dockButton->setText("Dock");
        ui.dockButton->setEnabled(false);
        ui.startButton->setText("Joystick Mode");
        ui.startButton->setEnabled(false);
        ui.joyButton->setText("Disable Joystick");
        ui.joyButton->setEnabled(true);
        ui.cancelButton->setEnabled(false);
        ui.statusDisplay->setText("Use joystick to control robot");
        break;
      case STATUS_ESTOPPED:
        ui.dockButton->setText("Dock");
        ui.dockButton->setEnabled(false);
        ui.startButton->setText("E-stopped");
        ui.startButton->setEnabled(false);
        ui.joyButton->setText("Joystick Unavailable");
        ui.joyButton->setEnabled(false);
        ui.cancelButton->setEnabled(false);
        ui.statusDisplay->setText("Robot is emergency stopped");
        break;
      case STATUS_DOCKED:
      {
        ui.dockButton->setText("Undock");
        ui.dockButton->setEnabled(true);
        if(qnode->getPreStopStatus() == STATUS_EXECUTING || qnode->getPreStopStatus() == STATUS_PLANNING)
        {
          ui.startButton->setText("Resume sweep");
          ui.cancelButton->setText("Cancel Sweep");
        }
        else
        {
          ui.startButton->setText("Start Sweep");
          ui.cancelButton->setText("New Room");
        }
        ui.joyButton->setText("Enable Joystick");
        ui.startButton->setEnabled(true);
        ui.joyButton->setEnabled(true);
        ui.cancelButton->setEnabled(true);
        ui.statusDisplay->setText("Robot is docked");
        break;
      }
      case STATUS_DOCKING:
        ui.dockButton->setText("Docking");
        ui.dockButton->setEnabled(false);
        ui.startButton->setEnabled(false);
        ui.joyButton->setEnabled(false);
        ui.cancelButton->setEnabled(false);
        ui.statusDisplay->setText("Robot is docking");
        break;
      case STATUS_UNDOCKING:
        ui.dockButton->setText("Undocking");
        ui.dockButton->setEnabled(false);
        ui.startButton->setEnabled(false);
        ui.joyButton->setEnabled(false);
        ui.cancelButton->setEnabled(false);
        ui.statusDisplay->setText("Robot is unocking");
        break;
      //case STATUS_RESET_OCT:
      //  ui.dockButton->setEnabled(false);
      //  ui.startButton->setEnabled(false);
      //  ui.joyButton->setEnabled(false);
      //  ui.cancelButton->setEnabled(false);
      //  ui.statusDisplay->setText("Resetting Octomap");
      //  break;
      default:
        ui.statusDisplay->setText("Invalid");
        break;
    }
  }
}


}  // namespace sweep_gui

