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
*********************************************************************/

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
