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
* Description: Implementation of the zoom viewer.
*********************************************************************/

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

