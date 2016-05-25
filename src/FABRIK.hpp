#ifndef FABRIK_HPP
#define FABRIK_HPP

#include "KinematicChain.hpp"
#include "Eigen/Dense"

class FABRIK : public InverseKinematic {
public:
  FABRIK();

  std::vector<double> solveIK(std::shared_ptr<KinematicChain> chain, int targetX, int targetY) override;
  void drawDebug(QPainter * painter) override;
  void drawCalculation(QPainter * painter, int iteration) override;
  int getSubclassIdentifier() const override {return 4;}

private:
  static const int EndEffector = -1;
  double tol;
  int iterations;

  Eigen::Vector3d calcJointPos(std::vector<double> thetas, std::vector<double> ls, int index) const;
  std::vector<double> calcThetas(std::vector<Eigen::Vector3d> jointPositions) const;

  std::vector<Eigen::Vector3d> jointPositions;
  std::vector<std::vector<Eigen::Vector3d>> jointPositionHistory;
  std::vector<double> diffs;
};

#endif
