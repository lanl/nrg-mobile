/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets>
#include <QApplication>
#include "../include/sweep_gui/demo_window.hpp"
#include "ros/ros.h"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv)
{
  // ros init
  ros::init(argc, argv, "sweep_gui");

  /*********************
  ** Qt
  **********************/
  QApplication app(argc, argv);

  // Show a splash screen
  QPixmap pm(":/images/splash.png");
  QSplashScreen splash(pm);
  splash.show();

  // Wait for show to finish, then process events
  //QTimer timer;
  //timer.setSingleShot(true);
  //QEventLoop loop;
  //connect(&splash, SIGNAL(splashShown()), &loop, SLOT(quit()));
  //connect(&timer, SIGNAL(timeout()), &loop, SLOT(quit()));
  //timer.start(1000);
  //loop.exec();

  // Start up main windows
  sweep_gui::DemoWindow w(argc,argv);
  w.show();
  splash.finish(&w);

  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
  return app.exec();
}
