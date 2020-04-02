#ifndef LED_HPP
#define LED_HPP

#include <QWidget>
#include <QPaintEvent>

class LedWidget : public QWidget
{
  Q_OBJECT

public:
  LedWidget(QWidget *parent = 0);

  QSize sizeHint();

public Q_SLOTS:
  void setState(bool on);

protected:
  void paintEvent(QPaintEvent *event);

  QColor circleColor;
};

#endif // LED_HPP
