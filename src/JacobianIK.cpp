#include "JacobianIK.hpp"

#include <cmath>
#include <iostream>

JacobianIK::JacobianIK() {
  numIterations = 0;
  maxIterations = 10000;
}


std::vector<double> JacobianIK::solveIK(std::shared_ptr<KinematicChain> chain, int targetX, int targetY) {
  Eigen::MatrixXd jacobian;
  Eigen::VectorXd deltaTheta;
  double alpha;
  std::vector<double> thetas;
  std::vector<double> ls;
  alphas.clear();
  alphas.shrink_to_fit();
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

  for(auto i = 0; i < maxIterations; i++) {
    e = targetPos - calcJointPos(thetas, ls, JacobianIK::EndEffector);
    norms.push_back(e.norm());
    // std::cerr << "\e[1;31mDEBUG:\e[0m " << "#" << i <<  " norm: " << e.norm() << std::endl;
    if(i > 0 && (preE-e).norm() < 0.02)
      break;

    if(e.norm() < 0.02)
      break;

    jacobian = calcJacobian(thetas, ls, targetPos);

    Eigen::Vector3d JJte = jacobian * jacobian.transpose() * e;
    alpha = e.dot(JJte) / JJte.dot(JJte);
    alphas.push_back(alpha);
    deltaTheta = alpha * jacobian.transpose() * e;

    for(size_t j = 0; j < thetas.size(); j++) {
      thetas[j] += deltaTheta(j);
    }
    positions.push_back(calcJointPos(thetas, ls, JacobianIK::EndEffector));

    preE = e;
    numIterations = i;
  }

  for(size_t i = 0; i < thetas.size(); i++) {
    // std::cerr << "\e[1;31mDEBUG:\e[0m " << "theta " << i << " " << thetas[i] << std::endl;
    if(thetas[i] != thetas[i])
      return std::vector<double>{0};
  }

  return thetas;
}

void JacobianIK::drawDebug(QPainter * painter) {
  painter->drawText(0, 0, QString("Iterations: %1").arg(numIterations));
  painter->drawRect(0, 20, 200, 100);
  double alphaPix = 200.0/alphas.size();
  double maxAlpha = 0;
  for (size_t i = 0; i < alphas.size(); i++) {
    maxAlpha = std::max(maxAlpha, alphas[i]);
  }
  double dAlpha = 100.0 / maxAlpha;

  for (size_t i = 0; i < alphas.size(); i++) {
    painter->drawLine(i*alphaPix, 120, i*alphaPix, 120-alphas[i]*dAlpha );
  }
  painter->drawText(0, 130, QString("max Alpha: %1").arg(maxAlpha));

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

void JacobianIK::drawCalculation(QPainter * painter, int iteration) {
  if(iteration > 0) {
    for(size_t i = 0; i < positions.size(); i++) {
      painter->drawEllipse(positions[i](0)-1, positions[i](1)-1, 2, 2);
    }
  }
}


Eigen::MatrixXd JacobianIK::calcJacobian(std::vector<double> thetas, std::vector<double> ls, Eigen::Vector3d target) const {
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

Eigen::Vector3d JacobianIK::calcJointPos(std::vector<double> thetas, std::vector<double> ls, int index) const {
  Eigen::Vector3d sumPos;
  sumPos << 0,0,0;
  double thetaSum = 0;

  int i = index;
  if(i == JacobianIK::EndEffector) i = thetas.size();

  for(auto j = 0; j < i; j++) {
    thetaSum += thetas[j];
    sumPos(0) += ls[j] * std::cos(thetaSum * M_PI / 180);
    sumPos(1) += ls[j] * std::sin(thetaSum * M_PI / 180);
  }

  return sumPos;
}
