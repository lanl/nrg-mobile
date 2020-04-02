/**
 * @file /src/editor_view.cpp
 *
 * @brief Implementation of the map editor viewer.
 *
 * @date October 2016
 **/

#include "sweep_gui/editor_view.hpp"

using namespace Qt;

EditorView::EditorView(QWidget* &qwidget) :
  QGraphicsView(qwidget),
  minScale(0.1),
  maxScale(10.0),
  editState(sweep_gui::EPAN),
  drawPathItem(0),
  drawRectItem(0),
  lineItem(0),
  textItem(0),
  drawStarted(false),
  drawSize(2.0),
  newImage(0)
{
  setDragMode(QGraphicsView::ScrollHandDrag);
  setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
}

void EditorView::setScene(QPixmap pMap)
{
  sceneSize = pMap.rect();
  QGraphicsScene *scene = new QGraphicsScene(sceneSize);
  scene->addPixmap(pMap);
  QGraphicsView::setScene(scene);
}

void EditorView::setScene(QGraphicsPixmapItem *pMap)
{
  sceneSize = QRect(pMap->pos().x(),pMap->pos().y(),pMap->pixmap().rect().width()*pMap->scale(), pMap->pixmap().rect().height()*pMap->scale());
  QGraphicsScene *scene = new QGraphicsScene(sceneSize);
  QGraphicsPixmapItem *item = scene->addPixmap(pMap->pixmap());
  item->setScale(pMap->scale());
  item->setPos(pMap->pos());
  QGraphicsView::setScene(scene);
}

QRect EditorView::getSize()
{
  return sceneSize;
}

void EditorView::showEvent(QShowEvent *e)
{
  fitInView(scene()->sceneRect(),Qt::KeepAspectRatio);
  minScale = 0.9 * curScale();
  maxScale = 10.0 * curScale();
}

void EditorView::wheelEvent(QWheelEvent *event)
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

void EditorView::mousePressEvent(QMouseEvent *event)
{
  if(event->button() == Qt::LeftButton)
  {
    QPointF click = mapToScene(event->pos());
    switch(editState)
    {
      case sweep_gui::ERASE:
        //fall through
      case sweep_gui::DRAW:
      {
        QPainterPath tempPath;
        tempPath.addRect(click.x() - drawSize/2.0, click.y() - drawSize/2.0, drawSize, drawSize);
        drawPath = drawPath.united(tempPath);
        if(drawStarted)
          scene()->removeItem(drawPathItem);
        else
          drawStarted = true;
        drawPathItem = scene()->addPath(drawPath,getPen(),getBrush());
        redoItemList.clear();
        break;
      }
      case sweep_gui::LINE:
      {
        linePos = QPointF(click);
        lineItem = scene()->addLine(click.x(),click.y(),click.x(),click.y(),getPen());
        redoItemList.clear();
        break;
      }
      case sweep_gui::TEXT:
      {
        if(textItem)
        {
          textItem->setTextInteractionFlags(Qt::NoTextInteraction);
          itemList.push_back(textItem);
          textItem = 0;
        }
        else
        {
          textItem = scene()->addText("Enter Text",QFont("Times", drawSize*2.0));
          textItem->setPos(click);
          textItem->setTextInteractionFlags(Qt::TextEditable);
          redoItemList.clear();
        }
        break;
      }
      case sweep_gui::EPAN:
        // fall through
      default:
        // do nothing
        break;
    }
  }

  // Call graphics view mouse press event
  QGraphicsView::mousePressEvent(event);
}

void EditorView::mouseReleaseEvent(QMouseEvent *event)
{
  if(event->button() == Qt::LeftButton)
  {
    if(sweep_gui::LINE == editState)
    {
      // finish line
      itemList.push_back(lineItem);
      lineItem = 0;
    }
    else if(sweep_gui::DRAW == editState || sweep_gui::ERASE == editState)
    {
      // end draw/erase
      if(drawStarted)
      {
        drawPath = QPainterPath();
        drawStarted = false;
        itemList.push_back(drawPathItem);
      }
    }
  }

  // Call graphics view mouse release event
  QGraphicsView::mouseReleaseEvent(event);
}

void EditorView::mouseMoveEvent(QMouseEvent *event)
{
  // Update mouse coordinate
  QPointF click = mapToScene(event->pos());
  Q_EMIT mouse(click.x(),click.y());

  // If we are drawing a line, update line position
  if(lineItem)
  {
    scene()->removeItem(lineItem);
    lineItem = scene()->addLine(linePos.x(),linePos.y(),click.x(),click.y(),getPen());
  }

  // If we are drawing, add squares
  if(drawStarted && event->buttons() == Qt::LeftButton)
  {
    QPainterPath tempPath;
    tempPath.addRect(click.x() - drawSize/2.0, click.y() - drawSize/2.0, drawSize, drawSize);
    drawPath = drawPath.united(tempPath);
    if(drawStarted)
      scene()->removeItem(drawPathItem);
    drawPathItem = scene()->addPath(drawPath,getPen(),getBrush());
  }

  // If we are in draw, erase or line mode, move draw rectangle
  if(drawRectItem && (sweep_gui::DRAW == editState || sweep_gui::ERASE == editState || sweep_gui::LINE == editState))
  {
    drawRectItem->setPos(click.x() - drawSize/2.0, click.y() - drawSize/2.0);
  }

  // Call graphics view mouse move event
  QGraphicsView::mouseMoveEvent(event);
}

void EditorView::scaleBy(qreal scaleFactor)
{
  qreal curScaleFactor = curScale();
  if (((curScaleFactor <= minScale) && (scaleFactor < 1.0)) ||
      ((curScaleFactor >= maxScale) && (scaleFactor > 1.0)))
  {
    return;
  }

  qreal sc = scaleFactor;
  if ((curScaleFactor * sc < minScale)&&(sc < 1.0))
  {
    sc = minScale / curScaleFactor;
  }
  else
  if ((curScaleFactor * sc > maxScale)&&(sc > 1.0))
  {
    sc = maxScale / curScaleFactor;
  }
  scale(sc, sc);
}

void EditorView::setEditState(sweep_gui::EditState es)
{
  // If we were drawing in the last state, finish the drawing
  if(drawStarted)
  {
    drawPath = QPainterPath();
    drawStarted = false;
    itemList.push_back(drawPathItem);
  }
  if(drawRectItem)
  {
    scene()->removeItem(drawRectItem);
    drawRectItem = 0;
  }

  editState = es;
  switch(es)
  {
    case sweep_gui::ERASE:
      // fall through
    case sweep_gui::DRAW:
      //fall through
    case sweep_gui::LINE:
    {
      QRect drawRect(0,0,drawSize,drawSize);
      drawRectItem = scene()->addRect(drawRect, QPen(QColor(Qt::black)), getBrush());
      drawRectItem->setZValue(1);
    }
      // fall through
    case sweep_gui::TEXT:
      setDragMode(QGraphicsView::NoDrag);
      break;
    case sweep_gui::PAN:
      // fall through
    default:
      setDragMode(QGraphicsView::ScrollHandDrag);
      break;
  }
}

QPen EditorView::getPen()
{
  if(sweep_gui::ERASE == editState)
    return QPen(QColor(Qt::white));
  else if(sweep_gui::LINE == editState)
    return QPen(QBrush(Qt::black),drawSize);
  else
    return QPen(QColor(Qt::black));
}

QBrush EditorView::getBrush()
{
  if(sweep_gui::ERASE == editState)
    return QBrush(QColor(Qt::white));
  else
    return QBrush(QColor(Qt::black));
}

void EditorView::setDrawSize(double size)
{
  drawSize = size;
  if(drawRectItem)
  {
    scene()->removeItem(drawRectItem);
    QRect drawRect(0,0,drawSize,drawSize);
    drawRectItem = scene()->addRect(drawRect, QPen(QColor(Qt::black)), getBrush());
    drawRectItem->setZValue(1);
  }
}

void EditorView::undo()
{
  if(itemList.size()>0)
  {
    scene()->removeItem(itemList.back());
    redoItemList.push_back(itemList.back());
    itemList.pop_back();
  }
}

void EditorView::redo()
{
  if(redoItemList.size()>0)
  {
    scene()->addItem(redoItemList.back());
    itemList.push_back(redoItemList.back());
    redoItemList.pop_back();
  }
}

void EditorView::loadImage(QPixmap im)
{
  if(!newImage)
  {
    setEditState(sweep_gui::EPAN);
    newImage = scene()->addPixmap(im);
    newImage->setFlag(QGraphicsItem::ItemIsMovable, true);
    newImage->setOpacity(0.3);
  }
}

void EditorView::scaleImage(double s)
{
  if(newImage)
  {
    newImage->setScale(s);
  }
}

void EditorView::finalizeImage()
{
  if(newImage)
  {
    scene()->removeItem(newImage);
    setScene(newImage);
    newImage = 0;
  }
}
