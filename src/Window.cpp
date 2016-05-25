#include "Window.hpp"
#include "IKWidget.hpp"

#include <QGridLayout>
#include <QTimer>
#include <QComboBox>
#include <QLabel>
#include <QCheckBox>

Window::Window() {
  setWindowTitle(tr("Inverse Kinematics Demo"));
  IKWidget *ik = new IKWidget(std::shared_ptr<KinematicChain>(new KinematicChain()), this);
  QLabel *solverLabel = new QLabel(tr("Algorithmus"));
  solverLabel->setAlignment(Qt::AlignHCenter);
  solverLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
  QComboBox *box = new QComboBox(this);
  box->addItem("Analytisch", 1);
  box->addItem("Transponierte Jacobi", 2);
  box->addItem("Pseudoinverse Jacobi", 3);
  box->addItem("FABRIK", 4);

  QLabel *numJointsLabel = new QLabel(tr("Anzahl Gelenke"));
  numJointsLabel->setAlignment(Qt::AlignHCenter);
  numJointsLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
  std::shared_ptr<QSlider> jointSlider = std::shared_ptr<QSlider>(new QSlider(Qt::Orientation::Horizontal, this));
  jointSlider->setMinimum(2);
  jointSlider->setMaximum(20);
  ik->setJointNumSlider(jointSlider);

  QCheckBox *enableComputationDraw = new QCheckBox(tr("Zeichne Berechnung"), this);
  QCheckBox *enableDebug = new QCheckBox(tr("Debug"), this);

  QLabel *spacer = new QLabel();

  QGridLayout *layout = new QGridLayout;
  QGridLayout *side = new QGridLayout;
  layout->addWidget(ik, 0, 0);

  side->addWidget(solverLabel, 0, 0);
  side->addWidget(box, 1, 0);
  side->addWidget(numJointsLabel, 2, 0);
  side->addWidget(jointSlider.get(), 3, 0);
  side->addWidget(enableComputationDraw, 4, 0);
  side->addWidget(enableDebug, 5, 0);
  side->addWidget(spacer, 6, 0);

  layout->addLayout(side, 0, 1);

  setLayout(layout);
  QTimer *timer = new QTimer(this);
  connect(timer, &QTimer::timeout, ik, &IKWidget::animate);
  connect(enableComputationDraw, &QCheckBox::stateChanged, ik, &IKWidget::changeDrawComputation);
  connect(enableDebug, &QCheckBox::stateChanged, ik, &IKWidget::changeDrawDebug);
  connect(box, static_cast<void(QComboBox::*)(int)>(&QComboBox::activated), ik, &IKWidget::selectSolver);
  connect(jointSlider.get(), &QSlider::valueChanged, ik, &IKWidget::changeJointNumber);
  timer->start(1000.0/60);
}
