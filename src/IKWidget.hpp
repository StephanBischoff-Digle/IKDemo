#ifndef IKWIDGET_HPP
#define IKWIDGET_HPP

#include <QWidget>
#include <QTime>
#include <QPen>
#include <QSlider>

#include "KinematicChain.hpp"

class IKWidget : public QWidget {
  Q_OBJECT
public:
  IKWidget(std::shared_ptr<KinematicChain> chain, QWidget *parent);

  void setJointNumSlider(std::shared_ptr<QSlider> slider) {jointNumSlider = slider;}

public slots:
  void animate();
  void selectSolver(int index);
  void changeJointNumber(int value);
  void changeDrawDebug(int state);
  void changeDrawComputation(int state);

protected:
  void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;
  void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;

private:
  void solve();

  std::shared_ptr<KinematicChain> chain;
  int screenX;
  int screenY;

  int mouseX;
  int mouseY;

  unsigned long frame;
  double debugCounter;

  bool drawDebug;
  bool drawComputation;

  QColor bgColor;
  QColor bugColor;
  QColor targetColor;
  QColor mainColor;
  QColor areaColor;

  QBrush bgBrush;
  QBrush bugBrush;
  QBrush mainBrush;
  QBrush areaBrush;

  QPen bugPen;
  QPen targetPen;
  QPen targetLinePen;
  QPen mainPen;
  QPen markPen;

  QFont bugFont;
  QTime timer;

  QTime computationTime;
  double computationMS;

  std::shared_ptr<QSlider> jointNumSlider;
};


#endif
