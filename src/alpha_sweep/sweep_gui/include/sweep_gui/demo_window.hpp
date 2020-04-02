/**
 * @file /include/sweep_gui/demo_window.hpp
 *
 * @brief Qt based gui for full coverage alpha sweep package.
 *
 * @date September 2016
 **/
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
