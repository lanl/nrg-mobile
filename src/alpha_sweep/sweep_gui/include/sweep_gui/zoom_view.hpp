/**
 * @file /include/sweep_gui/zoom_view.hpp
 *
 * @brief Qt Graphics Viewer that has a zoom function.
 *
 * @date September 2016
 **/
#ifndef sweep_gui_ZOOM_VIEW_H
#define sweep_gui_ZOOM_VIEW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QGraphicsView>
#include <QGraphicsPixmapItem>
#include <QWheelEvent>
#include <math.h>
#include "sweep_gui_enums.h"
#include "geometry_msgs/Pose.h"

/**
 * @brief Qt central, all operations relating to the view part here.
 */
class ZoomView : public QGraphicsView {
Q_OBJECT
public:
  ZoomView(QWidget*& qwidget);
  ~ZoomView();
  void setScene(std::vector<int8_t> &data, int dimx, int dimy, geometry_msgs::Pose mp, double res);
  void wheelEvent(QWheelEvent *event);
  void showEvent(QShowEvent *event);
  void mouseMoveEvent(QMouseEvent *event);
  QPixmap* getPixMap();
  void setZoom(QRectF zoom);
  void scaleBy(qreal scaleFactor);
Q_SIGNALS:
  void mouse(int x, int y);
public Q_SLOTS:
  void setScene(QPixmap pMap);
private:
  void zoomTo(QRectF zoom);
  QRectF startZoom;
  bool newZoom;
  double currentScale;
  double minScale;
  double maxScale;
  QPixmap *pixMap;
  QGraphicsItem *pixMapItem;
  QGraphicsScene *gScene;
};

#endif // sweep_gui_ZOOM_VIEW_H

