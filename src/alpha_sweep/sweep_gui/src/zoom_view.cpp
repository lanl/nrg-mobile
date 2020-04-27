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


#include "../include/sweep_gui/zoom_view.hpp"

using namespace Qt;

ZoomView::ZoomView(QWidget* &qwidget) :
  QGraphicsView(qwidget),
  currentScale(1.0),
  minScale(0.0001),
  maxScale(1000.0),
  pixMap(0),
  pixMapItem(0),
  gScene(0),
  newZoom(false)
{
  setDragMode(QGraphicsView::ScrollHandDrag);
  setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
  scale(1,-1);
}

ZoomView::~ZoomView()
{
  if(pixMapItem)
    delete pixMapItem;
  if(pixMap)
    delete pixMap;
  if(gScene)
    delete gScene;
}

void ZoomView::setScene(std::vector<int8_t> &data, int dimx, int dimy, geometry_msgs::Pose mp, double res)
{
  QImage *qi = new QImage(dimx, dimy, QImage::Format_Indexed8);
  QVector<QRgb> my_table;
  for(int i = 255; i >=0; i--)
  {
    my_table.push_back(qRgb(i,i,i));
  }
  qi->setColorTable(my_table);
  for(int i = 0; i<dimy; i++)
  {
    for(int j = 0; j<dimx; j++)
    {
      unsigned int tempPix;
      if(data[i*dimx+j] <40)
        tempPix = 0;
      else
        tempPix = 255;
      qi->setPixel(j,i,tempPix);
    }
  }

  if(pixMap)
    delete pixMap;

  pixMap = new QPixmap(dimx,dimy);
  pixMap->convertFromImage(*qi);
  delete qi;

  if(gScene && pixMapItem)
  {
    gScene->removeItem(pixMapItem);
    delete pixMapItem;
  }
  else
  {
    gScene = new QGraphicsScene(mp.position.x,mp.position.y,dimx*res,dimy*res,this);
    QGraphicsView::setScene(gScene);
    QGraphicsView::setSceneRect(gScene->sceneRect());
  }
  gScene->setSceneRect(mp.position.x,mp.position.y,dimx*res,dimy*res);
  pixMapItem = gScene->addPixmap(*pixMap);
  pixMapItem->setPos(mp.position.x,mp.position.y);
  pixMapItem->setScale(res);
  QGraphicsView::setSceneRect(gScene->sceneRect());
  if(!gScene->items().empty())
    pixMapItem->stackBefore(gScene->items().back());
}

void ZoomView::setScene(QPixmap pMap)
{
  if(pixMap)
    delete pixMap;

  pixMap = new QPixmap(pMap);
  if(pixMapItem)
  {
    gScene->removeItem(pixMapItem);
    delete pixMapItem;
  }
  pixMapItem = gScene->addPixmap(pMap);
  if(!gScene->items().empty())
    pixMapItem->stackBefore(gScene->items().back());
}

QPixmap* ZoomView::getPixMap()
{
  return pixMap;
}

void ZoomView::setZoom(QRectF zoom)
{
  if(!gScene || !scene())
  {
    std::cout << "Zoom view set zoom called before scene was set" << std::endl;
    return;
  }
  if(startZoom.isValid()) // if we have already set a start zoom previously, go straight to zoom function
  {
    zoomTo(zoom);
  }
  else // if not, wait for show event
  {
    startZoom = zoom;
    newZoom = true;
  }
}

void ZoomView::zoomTo(QRectF zoom)
{
  QPolygonF gBox = mapToScene(viewport()->frameGeometry());
  if(gBox.size()>2)
  {
    std::cout << "gbox size valid" << std::endl;
    qreal sf = 1;
    if(startZoom.isValid())
    {
      std::cout << "setting valid zoom" << std::endl;
      sf = fabs(gBox.at(1).y() - gBox.at(2).y())/startZoom.height();
      centerOn(QPointF(startZoom.x(),startZoom.y()));
    }
    else
    {
      QRectF sRect = gScene->sceneRect();
      sf = fabs(gBox.at(1).y() - gBox.at(2).y())/sRect.height();
    }
    scale(sf,sf);
    currentScale = 1.0;
    minScale = 0.05;
    maxScale = 10.0;
  }
}

void ZoomView::showEvent(QShowEvent *event)
{
  if(newZoom)
  {
    zoomTo(startZoom);
    newZoom = false;
  }
  QGraphicsView::showEvent(event);
}

void ZoomView::wheelEvent(QWheelEvent *event)
{
  if (event->modifiers() == Qt::ControlModifier)
  {
    scaleBy(pow(4.0 / 3.0, (event->delta() / 240.0)));
  }
  else
  {
    QGraphicsView::wheelEvent(event);
  }
}

void ZoomView::mouseMoveEvent(QMouseEvent *event)
{
  // Update mouse coordinate
  QPointF tempP = mapToScene(event->pos());
  Q_EMIT mouse(tempP.x(),tempP.y());

  // Call graphics view mouse move event
  QGraphicsView::mouseMoveEvent(event);
}

void ZoomView::scaleBy(qreal scaleFactor)
{
  if (((currentScale <= minScale) && (scaleFactor < 1.0)) ||
      ((currentScale >= maxScale) && (scaleFactor > 1.0)))
  {
    return;
  }

  qreal sc = scaleFactor;
  if ((currentScale * sc < minScale)&&(sc < 1.0))
  {
    sc = minScale / currentScale;
  }
  else if ((currentScale * sc > maxScale)&&(sc > 1.0))
  {
    sc = maxScale / currentScale;
  }
  currentScale *= sc;
  scale(sc, sc);
}

