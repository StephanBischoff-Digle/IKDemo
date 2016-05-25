#ifndef JACOBIANIK_HPP
#define JACOBIANIK_HPP

#include "KinematicChain.hpp"
#include "Eigen/Dense"

class JacobianIK : public InverseKinematic {
public:
  JacobianIK();

  std::vector<double> solveIK(std::shared_ptr<KinematicChain> chain, int targetX, int targetY) override;
  void drawDebug(QPainter * painter) override;
  void drawCalculation(QPainter * painter, int iteration) override;
  int getSubclassIdentifier() const override {return 2;}

private:
  static const int EndEffector = -1;

  Eigen::MatrixXd calcJacobian(std::vector<double> thetas, std::vector<double> ls, Eigen::Vector3d target) const;
  Eigen::Vector3d calcJointPos(std::vector<double> thetas, std::vector<double> ls, int index) const;

  int numIterations;
  int maxIterations;
  std::vector<double> alphas;
  std::vector<double> norms;
  std::vector<Eigen::Vector3d> positions;

};

#endif
