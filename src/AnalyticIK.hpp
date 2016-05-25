#ifndef ANALYTICIK_HPP
#define ANALYTICIK_HPP

#include "KinematicChain.hpp"

class AnalyticIK : public InverseKinematic {
  std::vector<double> solveIK(std::shared_ptr<KinematicChain> chain, int targetX, int targetY) override;
  void drawDebug(QPainter * painter) override;
  void drawCalculation(QPainter * painter, int iteration) override;
  int getSubclassIdentifier() const override {return 1;}

private:
  double t11;
  double t21;
  double t12;
  double t22;
  double l1;
  double l2;
};

#endif
