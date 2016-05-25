#ifndef JACOBIANPSEUDOIK_HPP
#define JACOBIANPSEUDOIK_HPP

#include "KinematicChain.hpp"
#include "Eigen/Dense"
#include "Eigen/SVD"


class PseudoInverseJacobi : public Eigen::JacobiSVD<Eigen::MatrixXd> {
public:
  PseudoInverseJacobi() {};
  PseudoInverseJacobi(Eigen::MatrixXd matrix) : Eigen::JacobiSVD<Eigen::MatrixXd>(matrix, Eigen::ComputeThinV | Eigen::ComputeThinU) {};

  void calcPseudoInverse(Eigen::MatrixXd& matrix) const;
};


class JacobianPseudoIK : public InverseKinematic {
public:
  JacobianPseudoIK();

  std::vector<double> solveIK(std::shared_ptr<KinematicChain> chain, int targetX, int targetY) override;
  void drawDebug(QPainter * painter) override;
  void drawCalculation(QPainter * painter, int iteration) override;
  int getSubclassIdentifier() const override {return 3;}

private:
  static const int EndEffector = -1;

  Eigen::MatrixXd calcJacobian(std::vector<double> thetas, std::vector<double> ls, Eigen::Vector3d target) const;
  Eigen::Vector3d calcJointPos(std::vector<double> thetas, std::vector<double> ls, int index) const;

  int numIterations;
  int maxIterations;
  std::vector<double> norms;
  std::vector<Eigen::Vector3d> positions;

};

#endif
