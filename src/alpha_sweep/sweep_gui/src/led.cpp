#include "../include/sweep_gui/led.hpp"
#include <QPainter>

using namespace Qt;

LedWidget::LedWidget(QWidget *parent)
  : QWidget(parent),
    circleColor(Qt::green)
{
  QSizePolicy policy;
  policy.setVerticalPolicy(QSizePolicy::Preferred);
  policy.setHeightForWidth(true);
  setSizePolicy(policy);
}

QSize LedWidget::sizeHint()
{
  return QSize(20,20);
}

void LedWidget::paintEvent(QPaintEvent *event)
{
  QPainter painter(this);
  painter.setPen(QColor(Qt::black));
  int px = (int)(width()/8.0);
  int py = -(int)(height()/8.0);
  int dist = (int)(height()/1.2);
  QRadialGradient radialGrad(QPointF(px,py),dist);
  radialGrad.setColorAt(0,Qt::white);
  radialGrad.setColorAt(0.2,circleColor);
  radialGrad.setColorAt(1,Qt::black);
  QBrush brush(radialGrad);
  brush.setStyle(Qt::RadialGradientPattern);
  painter.setBrush(brush);
  painter.translate(width()/2, height()/2);
  int rx = (int)(width()/2.5);
  int ry = (int)(height()/2.5);
  painter.drawEllipse(QPoint(0,0),rx,ry);
}

void LedWidget::setState(bool on)
{
  if(on)
    circleColor = QColor(Qt::green);
  else
    circleColor = QColor(Qt::red);
  update();
}
