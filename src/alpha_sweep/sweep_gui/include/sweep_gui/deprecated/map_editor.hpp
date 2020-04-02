#ifndef MAP_EDITOR_H
#define MAP_EDITOR_H

#include <QMainWindow>

namespace Ui {
class MapEditor;
}

class MapEditor : public QMainWindow
{
  Q_OBJECT

public:
  explicit MapEditor(QWidget *parent = 0);
  ~MapEditor();
  void setScene(QPixmap* pMap);

Q_SIGNALS:
  void newScene(QPixmap pMap);

public Q_SLOTS:
  /******************************************
  ** Auto-connections (connectSlotsByName())
  *******************************************/
  void on_panButton_released();
  void on_lineButton_released();
  void on_eraseButton_released();
  void on_drawButton_released();
  void on_textButton_released();
  void on_loadButton_released();
  void on_saveButton_released();
  void on_applyButton_released();
  void on_sizeSlider_valueChanged();
  void on_undoButton_released();
  void on_redoButton_released();
  void on_scaleSlider_valueChanged();
  void on_scaleApply_released();
private:
  QPixmap getPixMap();
  void showScaleTools(bool toShow);

  Ui::MapEditor *ui;
  QRect mapRect;
  bool newImage;
};

#endif // MAP_EDITOR_H
