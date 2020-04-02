#include "sweep_gui/map_editor.hpp"
#include "ui_map_editor.h"
#include "sweep_gui/sweep_gui_enums.h"
#include <QtGui>

MapEditor::MapEditor(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MapEditor),
  newImage(false)
{
  ui->setupUi(this);

  ui->sizeSlider->setRange(1.0,20.0);
  ui->sizeSlider->setValue(4.0);
  showScaleTools(false);
}

MapEditor::~MapEditor()
{
  delete ui;
}

void MapEditor::setScene(QPixmap* pMap)
{
  ui->graphicsView->setScene(*pMap);
  mapRect = pMap->rect();
}

void MapEditor::on_panButton_released()
{
  ui->graphicsView->setEditState(sweep_gui::EPAN);
}

void MapEditor::on_lineButton_released()
{
  ui->graphicsView->setEditState(sweep_gui::LINE);
}

void MapEditor::on_eraseButton_released()
{
  ui->graphicsView->setEditState(sweep_gui::ERASE);
}

void MapEditor::on_drawButton_released()
{
  ui->graphicsView->setEditState(sweep_gui::DRAW);
}

void MapEditor::on_textButton_released()
{
  ui->graphicsView->setEditState(sweep_gui::TEXT);
}

void MapEditor::on_loadButton_released()
{
  if(!newImage)
  {
    QString fileName = QFileDialog::getOpenFileName(this, tr("Load Map Image"), "~/Documents", tr("Image Files (*.png)"));
    QPixmap image(fileName);
    ui->graphicsView->loadImage(image);
    ui->loadButton->setText("Finalize Map");
    ui->saveButton->setEnabled(false);
    ui->applyButton->setEnabled(false);
    showScaleTools(true);
    newImage = true;
  }
  else
  {
    ui->graphicsView->finalizeImage();
    ui->loadButton->setText("Load Map");
    ui->saveButton->setEnabled(true);
    ui->applyButton->setEnabled(true);
    showScaleTools(false);
    newImage = false;
  }
}

void MapEditor::showScaleTools(bool toShow)
{
  if(toShow)
  {
    ui->scaleSlider->setValue(50);
    ui->scaleEdit->setText("1.0");
    ui->scaleSlider->show();
    ui->scaleApply->show();
    ui->scaleEdit->show();
    ui->scaleLabel->show();
  }
  else
  {
    ui->scaleSlider->hide();
    ui->scaleApply->hide();
    ui->scaleEdit->hide();
    ui->scaleLabel->hide();
  }
}

void MapEditor::on_scaleSlider_valueChanged()
{
  if(newImage)
  {
    double dScale = ui->scaleSlider->value()/50.0;
    QString sScale;
    sScale.setNum(dScale,'f',2);
    ui->scaleEdit->setText(sScale);
    ui->graphicsView->scaleImage(dScale);
  }
}

void MapEditor::on_scaleApply_released()
{
  if(newImage)
  {
    ui->scaleSlider->blockSignals(true);
    ui->scaleSlider->setValue(ui->scaleEdit->text().toDouble()*50.0);
    ui->graphicsView->scaleImage(ui->scaleEdit->text().toDouble());
    ui->scaleSlider->blockSignals(false);
  }
}

void MapEditor::on_saveButton_released()
{
  QString fileName = QFileDialog::getSaveFileName(this, tr("Save Map Image"), "~/Documents/", tr("Image Files (*.png)"));
  if(!fileName.contains("."))
    fileName.append(".png");

  getPixMap().save(fileName);
}

void MapEditor::on_applyButton_released()
{
  Q_EMIT newScene(getPixMap());
}

QPixmap MapEditor::getPixMap()
{
  QRect pSize = ui->graphicsView->getSize();
  QPixmap *pMap = new QPixmap(pSize.width(), pSize.height());
  pMap->fill();
  QPainter painter(pMap);
  qreal x1,y1,x2,y2;
  QPoint p1, p2;
  ui->graphicsView->sceneRect().getCoords(&x1,&y1,&x2,&y2);
  p1 = ui->graphicsView->mapFromScene(QPointF(x1,y1));
  p2 = ui->graphicsView->mapFromScene(QPointF(x2,y2));
  ui->graphicsView->render(&painter,QRectF(pSize), QRect(p1,p2));
  painter.end();

  return *pMap;
}

void MapEditor::on_sizeSlider_valueChanged()
{
  ui->graphicsView->setDrawSize(ui->sizeSlider->value() / 2.0);
}

void MapEditor::on_undoButton_released()
{
  ui->graphicsView->undo();
}

void MapEditor::on_redoButton_released()
{
  ui->graphicsView->redo();
}
