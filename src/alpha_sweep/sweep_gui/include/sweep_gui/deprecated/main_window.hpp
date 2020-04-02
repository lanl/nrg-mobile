/**
 * @file /include/sweep_gui/main_window.hpp
 *
 * @brief Qt based gui for full coverage alpha sweep package.
 *
 * @date September 2016
 **/
#ifndef sweep_gui_MAIN_WINDOW_H
#define sweep_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <sstream>
#include <cmath>
#include "sweep_gui_enums.h"
#include "cell_bounds.h"
#include "sweep_gui/map_editor.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace sweep_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
  MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

  void closeEvent(QCloseEvent *event); // Overloaded function

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
  void on_action_Preferences_triggered();
  void on_action_Save_triggered();
  void on_action_Load_triggered();
  void on_action_Editor_triggered();
  void on_startButton_released();
  void on_mapSweepButton_released();
  void on_panTool_released();
  void on_planFromPosTool_released();
  void on_planFromClickTool_released();
  void on_addFromClickTool_released();
  void on_removePosesTool_released();
  void on_clearButton_released();
  void on_stopButton_released();
  void on_clearPlanButton_released();
  void on_showPlanButton_released();
  void on_cancelButton_released();

  /*****************************************
   ** Manual connections
   *****************************************/
  void showPlan(bool addToPlan);
  void updateCoords(int x, int y);
  void newPose();
  void newTarget();
  void updateMap();
  void receiveStatus();
  void addPlan(QPointF p);
  void addPose(QPointF p1, QPointF p2);
  void removePoses(QPointF p1, QPointF p2);
  void showUnclearedMessage();

private:
	Ui::MainWindowDesign ui;
  QNode qnode;
  double mapResolution;
  double mapOriginX;
  double mapOriginY;
  double mapWidth;
  double mapHeight;
  double robotWidth;
  double robotHeight;

  // plan
  std::vector<geometry_msgs::Pose> plan;
  bool retryAvailable;
  bool autoSweep;

  // graphics items
  QGraphicsScene *scene;
  QPainterPath vPath;
  QGraphicsItem *vPathItem;
  QPainterPath robotPath;
  QGraphicsItem *robotPathItem;
  std::vector<QGraphicsLineItem*> planLines;
  std::vector<QGraphicsPathItem*> planArrows;
  QGraphicsPathItem* targetArrow;
  bool targetDrawn;
  bool robotDrawn;
  bool planDrawn;
  bool planArrowsDrawn;

  QPainterPath makeArrow(geometry_msgs::Pose pose, double length);
  void changeStatus(Status stat);
  Status status;
  void hidePlan();

  // Map editor
  MapEditor* me;
};

}  // namespace sweep_gui

#endif // sweep_gui_MAIN_WINDOW_H
