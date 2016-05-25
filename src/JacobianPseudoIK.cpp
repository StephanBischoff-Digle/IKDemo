#include "JacobianPseudoIK.hpp"

#include <cmath>
#include <iostream>


JacobianPseudoIK::JacobianPseudoIK() {
  numIterations = 0;
  maxIterations = 10000;
}


std::vector<double> JacobianPseudoIK::solveIK(std::shared_ptr<KinematicChain> chain, int targetX, int targetY) {
  Eigen::MatrixXd jacobian;
  Eigen::VectorXd deltaTheta;
  std::vector<double> thetas;
  std::vector<double> ls;

  norms.clear();
  norms.shrink_to_fit();
  positions.clear();
  positions.shrink_to_fit();

  for (size_t i = 0; i < chain->size(); i++) {
    thetas.push_back(chain->getJoint(i)->getTheta());
    ls.push_back(chain->getJoint(i)->getLinkLength());
  }

  Eigen::Vector3d targetPos(targetX, targetY, 0);
  Eigen::Vector3d e;
  Eigen::Vector3d preE;
  PseudoInverseJacobi j;
  double tmpNorm;

  for(auto i = 0; i < maxIterations; i++) {
    e = targetPos - calcJointPos(thetas, ls, JacobianPseudoIK::EndEffector);
    tmpNorm = e.norm();
    norms.push_back(tmpNorm);
    // std::cerr << "\e[1;31mDEBUG:\e[0m " << "#" << i <<  " norm: " << e.norm() << std::endl;
    if(i > 0 && (preE-e).norm() < 0.02)
      break;

    if(tmpNorm < 0.02)
      break;

    // std::cerr << "\e[1;31mDEBUG:\e[0m " << "Calculate jacobian" << std::endl;
    jacobian = calcJacobian(thetas, ls, targetPos);
    // std::cerr << "\e[1;31mDEBUG:\e[0m " << "Setup SVD" << std::endl;
    j = PseudoInverseJacobi(jacobian);
    // std::cerr << "\e[1;31mDEBUG:\e[0m " << "Calculate pseudo inverse" << std::endl;
    j.calcPseudoInverse(jacobian);

    deltaTheta = jacobian * e;

    for(size_t k = 0; k < thetas.size(); k++)
      thetas[k] += deltaTheta(k);

    positions.push_back(calcJointPos(thetas, ls, JacobianPseudoIK::EndEffector));

    preE = e;
    numIterations = i;
  }

  // std::cerr << "\e[1;31mDEBUG:\e[0m " << "---------------------------------" << std::endl;
  for(size_t i = 0; i < thetas.size(); i++) {
    // std::cerr << "\e[1;31mDEBUG:\e[0m " << "theta " << i << " " << thetas[i] << std::endl;
    if(thetas[i] != thetas[i])
      return std::vector<double>{0};
  }

  return thetas;
}

void JacobianPseudoIK::drawDebug(QPainter * painter) {
  painter->drawText(0, 0, QString("Iterations: %1").arg(numIterations));

  painter->drawRect(0, 140, 200, 100);
  double normPix = 200.0/norms.size();
  double maxNorm = 0;
  for (size_t i = 0; i < norms.size(); i++) {
    maxNorm = std::max(maxNorm, norms[i]);
  }
  double dNorm = 100.0 / maxNorm;
  for (size_t i = 0; i < norms.size(); i++) {
    painter->drawLine(i*normPix, 240, i*normPix, 240-norms[i]*dNorm );
  }
  painter->drawText(0, 250, QString("max e.norm(): %1").arg(maxNorm));
}

void JacobianPseudoIK::drawCalculation(QPainter * painter, int iteration) {
  if(iteration > 0) {
    for(size_t i = 0; i < positions.size(); i++) {
      painter->drawEllipse(positions[i](0)-1, positions[i](1)-1, 2, 2);
    }
  }
}


Eigen::MatrixXd JacobianPseudoIK::calcJacobian(std::vector<double> thetas, std::vector<double> ls, Eigen::Vector3d target) const {
  Eigen::MatrixXd jacobian(3, thetas.size());
  Eigen::Vector3d tmp;

  for(size_t i = 0; i < thetas.size(); i++) {
    tmp = Eigen::Vector3d::UnitZ().cross(target - calcJointPos(thetas, ls, i));
    jacobian(0, i) = tmp(0);
    jacobian(1, i) = tmp(1);
    jacobian(2, i) = tmp(2);
  }

  return jacobian;
}

Eigen::Vector3d JacobianPseudoIK::calcJointPos(std::vector<double> thetas, std::vector<double> ls, int index) const {
  Eigen::Vector3d sumPos;
  sumPos << 0,0,0;
  double thetaSum = 0;

  int i = index;
  if(i == JacobianPseudoIK::EndEffector) i = thetas.size();

  for(auto j = 0; j < i; j++) {
    thetaSum += thetas[j];
    sumPos(0) += ls[j] * std::cos(thetaSum * M_PI / 180);
    sumPos(1) += ls[j] * std::sin(thetaSum * M_PI / 180);
  }

  return sumPos;
}


void PseudoInverseJacobi::calcPseudoInverse(Eigen::MatrixXd& matrix) const {
  eigen_assert(m_isInitialized && "SVD is not initialized.");
  double pinvtol = 1.e-6;
  SingularValuesType singulars_inv = m_singularValues;
  for(auto i = 0; i < m_workMatrix.cols(); ++i) {
    if(m_singularValues(i) > pinvtol)
      singulars_inv(i) = 1.0/m_singularValues(i);
    else
      singulars_inv(i) = 0;
  }
  matrix = (m_matrixV * singulars_inv.asDiagonal() * m_matrixU.transpose());
}
