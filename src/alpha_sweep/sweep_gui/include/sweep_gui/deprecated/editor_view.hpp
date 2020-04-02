/**
 * @file /include/sweep_gui/editor_view.hpp
 *
 * @brief Qt Graphics Viewer that has a editor function.
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

/**
 * @brief Qt central, all operations relating to the view part here.
 */
class EditorView : public QGraphicsView {
Q_OBJECT
public:
  EditorView(QWidget*& qwidget);
  void setScene(QPixmap pMap);
  void setScene(QGraphicsPixmapItem *pMap);
  void wheelEvent(QWheelEvent *event);
  void mouseMoveEvent(QMouseEvent *event);
  void mousePressEvent(QMouseEvent *event);
  void mouseReleaseEvent(QMouseEvent *event);
  void setEditState(sweep_gui::EditState es);
  void showEvent(QShowEvent *e);
  QRect getSize();
  void setDrawSize(double size);
  void undo();
  void redo();
  void loadImage(QPixmap im);
  void finalizeImage();
  void scaleImage(double s);
Q_SIGNALS:
  void mouse(int x, int y);
  void planClick(QPointF p);
  void poseClick(QPointF p1, QPointF p2);
  void removeBox(QPointF p1, QPointF p2);
private:
  double minScale;
  double maxScale;
  sweep_gui::EditState editState;
  QPointF mousePressPos;
  QPainterPath drawPath;
  QGraphicsItem *drawPathItem;
  QGraphicsItem *drawRectItem;
  QGraphicsItem *lineItem;
  QGraphicsTextItem *textItem;
  std::vector<QGraphicsItem*> itemList;
  std::vector<QGraphicsItem*> redoItemList;
  QPointF linePos;
  bool drawStarted;
  QRect sceneSize;
  double drawSize;
  QGraphicsPixmapItem *newImage;

  inline qreal curScale() const
  {
    return transform().m11();
  }
  void scaleBy(qreal scaleFactor);
  QPen getPen();
  QBrush getBrush();
};

#endif // sweep_gui_ZOOM_VIEW_H

