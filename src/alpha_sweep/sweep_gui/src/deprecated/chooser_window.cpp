/**
 * @file /src/chooser_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include "../include/sweep_gui/chooser_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace sweep_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [ChooserWindow]
*****************************************************************************/

ChooserWindow::ChooserWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent)
  , init_argc(argc)
  , init_argv(argv)
{
  dw = 0;
  mw = 0;
  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
}

ChooserWindow::~ChooserWindow() {
  if(dw)
    delete dw;
  if(mw)
    delete mw;
}

/*
 * Command buttons released
 */
void ChooserWindow::on_demoButton_clicked()
{
  if(dw)
    delete dw;
  dw = new DemoWindow(init_argc, init_argv, this);
  dw->show();
  hide();
  dw->raise();
}

void ChooserWindow::on_debugButton_clicked()
{
  if(mw)
    delete mw;
  mw = new MainWindow(init_argc, init_argv, this);
  mw->show();
  hide();
  mw->raise();
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void ChooserWindow::closeEvent(QCloseEvent *event)
{
  // Add things we need to do on close here
  QMainWindow::closeEvent(event);
}

}  // namespace sweep_gui

