/**
 * @file /include/sweep_gui/chooser_window.hpp
 *
 * @brief Qt based gui for full coverage alpha sweep package.
 *
 * @date September 2016
 **/
#ifndef sweep_gui_CHOOSER_WINDOW_H
#define sweep_gui_CHOOSER_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_chooser_window.h"
#include "main_window.hpp"
#include "demo_window.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace sweep_gui {

/*****************************************************************************
** Interface [ChooserWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class ChooserWindow : public QMainWindow {
Q_OBJECT

public:
  ChooserWindow(int argc, char** argv, QWidget *parent = 0);
  ~ChooserWindow();

  void closeEvent(QCloseEvent *event); // Overloaded function

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
  void on_demoButton_clicked();
  void on_debugButton_clicked();

private:
  int init_argc;
  char** init_argv;
  Ui::ChooserWindowDesign ui;
  MainWindow* mw;
  DemoWindow* dw;

};

}  // namespace sweep_gui

#endif // sweep_gui_CHOOSER_WINDOW_H
