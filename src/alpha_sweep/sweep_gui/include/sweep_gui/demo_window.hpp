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

#ifndef sweep_gui_DEMO_WINDOW_H
#define sweep_gui_DEMO_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include "ui_demo_window.h"
#include "qnode.hpp"
#include <sstream>
#include <cmath>
#include "sweep_gui_enums.h"
#include "cell_bounds.h"
#include "full_coverage/DebugGrid.h"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace sweep_gui {

/*****************************************************************************
** Interface [DemoWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class DemoWindow : public QMainWindow {
Q_OBJECT

public:
  DemoWindow(int argc, char** argv, QWidget *parent = 0);
  ~DemoWindow();

  void closeEvent(QCloseEvent *event); // Overloaded function
  bool isInit();

public Q_SLOTS:
  /******************************************
  ** Auto-connections (connectSlotsByName())
  *******************************************/
  void on_action_Preferences_triggered();
  void on_startButton_clicked();
  void on_joyButton_clicked();
  void on_cancelButton_clicked();
  void on_zMinusButton_clicked();
  void on_zPlusButton_clicked();
  void on_zoomResetButton_clicked();
  void on_centerBox_stateChanged(int state);
  void on_quitButton_clicked();
  void on_dockButton_clicked();

  /*****************************************
   ** Manual connections
   *****************************************/
  void showPlan();
  //void updateCoords(int x, int y);
  void newPose();
  void newTarget();
  void newNavPose();
  bool updateMap();
  void receiveStatus();
  void showUnclearedMessage();
  void showJoyMessage();
  void showEStopMessage();
  void showDoneMessage();
  void showNotInitMessage();
  void showDebugInfo();
  void saveMap(QString fileName);

private:
  Ui::DemoWindowDesign ui;
  QNode *qnode;
  double mapResolution;
  double mapOriginX;
  double mapOriginY;
  double mapWidth;
  double mapHeight;
  double robotWidth;
  double robotHeight;

  // plan
  std::vector<geometry_msgs::Pose> plan;

  // graphics items
  QGraphicsScene *scene;
  QPainterPath vPath;
  QGraphicsItem *vPathItem;
  QPainterPath robotPath;
  QGraphicsItem *robotPathItem;
  std::vector<QGraphicsLineItem*> planLines;
  std::vector<QGraphicsPathItem*> planArrows;
  QGraphicsPathItem* targetArrow;
  QGraphicsPathItem* navArrow;
  QGraphicsPathItem* dockArrow;
  bool robotDrawn;
  bool centerOnRob;
  double lastTheta;
  std::vector<QGraphicsPathItem*> debugPolygons;

  QPainterPath makeArrow(geometry_msgs::Pose pose, double length);
  void changeStatus(Status stat);
  Status status;
  void hidePlan();
  bool init;
  bool hasMap;

  // sensor info
  double sensorW;
  double sensorL;
  double sensorX;
  double sensorY;

};

}  // namespace sweep_gui

#endif // sweep_gui_DEMO_WINDOW_H
