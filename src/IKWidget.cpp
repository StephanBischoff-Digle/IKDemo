#include "IKWidget.hpp"
#include "AnalyticIK.hpp"
#include "JacobianIK.hpp"
#include "JacobianPseudoIK.hpp"
#include "FABRIK.hpp"

#include <QPainter>
#include <QPaintEvent>

#include <cmath>
#include <iostream>

IKWidget::IKWidget(std::shared_ptr<KinematicChain> chain, QWidget *parent)
  : QWidget(parent) {
  this->chain = chain;

  bgColor     = QColor(0x55, 0x55, 0x55);
  bugColor    = QColor(0xe2, 0x8a, 0x34);
  targetColor = QColor(0x8a, 0xe2, 0x34);
  mainColor   = QColor(0xff, 0xff, 0xff);
  areaColor   = QColor(0x44, 0x44, 0x44);

  bgBrush = QBrush(bgColor);
  bugBrush = QBrush(bugColor);
  mainBrush = QBrush(mainColor);
  areaBrush = QBrush(areaColor);

  bugPen = QPen(bugColor);
  targetPen = QPen(targetColor);
  targetLinePen = QPen(targetColor, 2, Qt::PenStyle::DotLine);
  mainPen = QPen(mainColor, 4);
  markPen = QPen(areaColor);

  bugFont = QFont("Input Mono");
  bugFont.setPixelSize(10);


  screenX = 1500;
  screenY = 1000;

  mouseX = 0;
  mouseY = 0;

  setFixedSize(screenX, screenY);
  frame = 0;
  debugCounter = 0;

  drawDebug = false;
  drawComputation = false;

  double length = (std::min(screenX,screenY)/2.0 - 10)/2.0;
  chain->addJoint(std::shared_ptr<Joint>(new Joint(0, length, 100)));
  chain->addJoint(std::shared_ptr<Joint>(new Joint(0, length, 100)));

  chain->setIK(std::shared_ptr<InverseKinematic>(new AnalyticIK()));
  // chain->setIK(std::shared_ptr<InverseKinematic>(new FABRIK()));

  computationTime.start();
  computationMS = 0;
}

void IKWidget::animate() {
  update();
}

void IKWidget::mousePressEvent(QMouseEvent *event) {
  mouseX = event->x() - screenX/2;
  mouseY = event->y() - screenY/2;

  solve();
}

void IKWidget::solve() {
  computationTime.restart();
  std::vector<double> thetas = chain->getIK()->solveIK(chain, mouseX, mouseY);
  computationMS = computationTime.elapsed();

  if(thetas.size() == chain->size()) {
    for (size_t i = 0; i < thetas.size(); i++) {
      chain->getJoint(i)->setTargetTheta(thetas[i]);
    }
  }
  chain->balanceSpeed();
}


void IKWidget::paintEvent(QPaintEvent *event) {
  QPainter painter;
  painter.begin(this);
  painter.setRenderHint(QPainter::Antialiasing);
  frame++;

  // start painting
  int elapsedMS = 0;
  double fps = 0;
  if(frame == 1) {
    timer.start();
    elapsedMS = 0;
  } else {
    fps = std::round(frame/(double(timer.elapsed())/1000.0));
    elapsedMS = timer.elapsed() / double(frame);
  }


  chain->updateChain(elapsedMS);

  // clip background
  painter.setBackground(bgBrush);
  painter.fillRect(event->rect(), bgBrush);

  // draw reachable area
  double maxReach = 0;
  for(size_t i = 0; i < chain->size(); i++) {
    maxReach += chain->getJoint(i)->getLinkLength();
  }
  // draw watermark
  painter.setPen(markPen);
  painter.setFont(bugFont);
  painter.drawText(screenX-120, 20, QString("Stephan Bischoff"));

  painter.save();
  painter.translate(screenX/2, screenY/2);
  painter.setBrush(areaBrush);
  painter.drawEllipse(-maxReach,-maxReach, 2*maxReach, 2*maxReach);
  painter.restore();

  if(drawDebug) {
    painter.setFont(bugFont);
    painter.setPen(bugPen);
    painter.drawText(QRect(40, 20, 200, 15), Qt::AlignLeft, QString("FPS: %1 | \u0394t: %2ms").arg(fps).arg(elapsedMS));
    painter.drawText(QRect(40, 30, 200, 15), Qt::AlignLeft, QString("computation time: %1ms").arg(computationMS));
    painter.save();
    painter.translate(40, 60);
    chain->getIK()->drawDebug(&painter);
    painter.restore();
  }

  painter.save();
  painter.translate(20, 23);
  painter.rotate(debugCounter += 90*elapsedMS/1000.0);
  painter.drawRect(-5, -5, 10, 10);
  painter.restore();
  painter.save();
  painter.translate(20, 23);
  painter.rotate(debugCounter*1.3);
  painter.drawRect(-3, -3, 6, 6);
  painter.restore();

  // set (0,0) to canvas center
  painter.translate(screenX/2, screenY/2);
  painter.save();

  painter.setPen(targetLinePen);
  painter.drawLine(0, 0, mouseX, mouseY);
  painter.setPen(targetPen);
  painter.drawEllipse(mouseX-5, mouseY-5, 10, 10);
  painter.drawLine(mouseX-5, mouseY-5, mouseX+4, mouseY+4);
  painter.drawLine(mouseX-5, mouseY+5, mouseX+4, mouseY-4);
  painter.restore();

  painter.save();
  painter.setPen(mainPen);
  painter.setBrush(mainBrush);
  for(typeof(chain->size()) i = 0; i < chain->size(); i++) {
    painter.rotate(chain->getJoint(i)->getTheta());
    painter.drawEllipse(-4, -4, 9, 9);
    painter.drawLine(0, 0, chain->getJoint(i)->getLinkLength(), 0);
    painter.translate(chain->getJoint(i)->getLinkLength(), 0);
  }
  painter.drawRect(-4, -4, 9, 9);


  painter.restore();
  painter.setPen(bugPen);
  if(drawComputation) chain->getIK()->drawCalculation(&painter, 10000);

  painter.end();
}

void IKWidget::selectSolver(int index) {
  switch(index) {
    case 0:
      if(chain->size() > 2) {
        chain = std::shared_ptr<KinematicChain>(new KinematicChain());
        double length = (std::min(screenX,screenY)/2.0 - 10)/2.0;
        chain->addJoint(std::shared_ptr<Joint>(new Joint(0, length, 100)));
        chain->addJoint(std::shared_ptr<Joint>(new Joint(0, length, 100)));
      }
      chain->setIK(std::shared_ptr<InverseKinematic>(new AnalyticIK()));
      jointNumSlider->setEnabled(false);
      jointNumSlider->setSliderPosition(2);
      break;
    case 1:
      chain->setIK(std::shared_ptr<InverseKinematic>(new JacobianIK()));
      jointNumSlider->setEnabled(true);
      break;
    case 2:
      chain->setIK(std::shared_ptr<InverseKinematic>(new JacobianPseudoIK()));
      jointNumSlider->setEnabled(true);
      break;
    case 3:
      chain->setIK(std::shared_ptr<InverseKinematic>(new FABRIK()));
      jointNumSlider->setEnabled(true);
      break;
    case 4:
      std::cerr << "\e[1;31mDEBUG:\e[0m " << "4" << std::endl;
      break;
  }
  solve();
}

void IKWidget::changeJointNumber(int value) {
  if(chain->getIK()->getSubclassIdentifier() != std::shared_ptr<InverseKinematic>(new AnalyticIK())->getSubclassIdentifier()) {
    std::shared_ptr<InverseKinematic> solver = chain->getIK();
    chain = std::shared_ptr<KinematicChain>(new KinematicChain());
    double length = (std::min(screenX,screenY)/2.0 - 10)/value;
    for(auto i = 0; i < value; i++) {
      chain->addJoint(std::shared_ptr<Joint>(new Joint(0, length, 400.0/value)));
    }
    chain->setIK(solver);
  }
  solve();
}

void IKWidget::changeDrawDebug(int state) {
  drawDebug = state == Qt::CheckState::Checked;
}
void IKWidget::changeDrawComputation(int state) {
  drawComputation = state == Qt::CheckState::Checked;
}
