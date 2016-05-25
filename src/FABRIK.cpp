#include "FABRIK.hpp"

#include <iostream>


FABRIK::FABRIK() {
  tol = 0.02;
  iterations = 0;
}


std::vector<double> FABRIK::solveIK(std::shared_ptr<KinematicChain> chain, int targetX, int targetY) {
  Eigen::Vector3d target;
  target << targetX, targetY, 0;
  std::vector<double> ls;
  std::vector<double> thetas;

  jointPositions.clear();
  jointPositions.shrink_to_fit();
  jointPositionHistory.clear();
  jointPositionHistory.shrink_to_fit();
  diffs.clear();
  diffs.shrink_to_fit();
  iterations = 0;

  double maxReach = 0;
  double tmp;

  for (size_t i = 0; i < chain->size(); i++) {
    tmp = chain->getJoint(i)->getLinkLength();
    maxReach += tmp;
    ls.push_back(tmp);
    thetas.push_back(chain->getJoint(i)->getTheta());
  }

  for(size_t i = 0; i < ls.size(); i++) {
    jointPositions.push_back(calcJointPos(thetas, ls, i));
  }
  jointPositions.push_back(calcJointPos(thetas, ls, FABRIK::EndEffector));

  double n = jointPositions.size()-1;
  double ri;
  double lambdai;

  // std::cerr << "\e[1;31mDEBUG:\e[0m " << "maxReach " << maxReach << std::endl;
  // std::cerr << "\e[1;31mDEBUG:\e[0m " << "targetNorm " << target.norm() << std::endl;

  if(maxReach < target.norm()) {
    for(auto i = 0; i < n; i++) {
      ri = (target - jointPositions[i]).norm();
      lambdai = ls[i] / ri;
      jointPositions[i+1] = (1-lambdai) * jointPositions[i] + lambdai * target;
    }
  } else {
    Eigen::Vector3d base = jointPositions[0];
    double dif = (jointPositions[n] - target).norm();
    diffs.push_back(dif);
    // std::cerr << "\e[1;31mDEBUG:\e[0m " << "init Dif " << dif << std::endl;
    while(dif > tol) {
      // forward reaching
      jointPositions[n] = target;
      for(auto i = n-1; i >= 0; i--) {
        ri = (jointPositions[i+1] - jointPositions[i]).norm();
        lambdai = ls[i] / ri;
        jointPositions[i] = (1 - lambdai) * jointPositions[i+1] + lambdai * jointPositions[i];
      }

      jointPositionHistory.push_back(jointPositions);

      // backward reaching
      jointPositions[0] = base;
      for(auto i = 0; i < n; i++) {
        ri = (jointPositions[i+1] - jointPositions[i]).norm();
        lambdai = ls[i] / ri;
        jointPositions[i+1] = (1-lambdai) * jointPositions[i] + lambdai * jointPositions[i+1];
      }

      jointPositionHistory.push_back(jointPositions);
      dif = (jointPositions[n] - target).norm();
      diffs.push_back(dif);
      iterations++;
      // std::cerr << "\e[1;31mDEBUG:\e[0m " << iterations << " dif " << dif << std::endl;
      if(iterations > 10000) break;
    }
  }
  // jointPositions.push_back(target);

  thetas = calcThetas(jointPositions);
  for(size_t i = 0; i < thetas.size(); i++) {
    // std::cerr << "\e[1;31mDEBUG:\e[0m " << "theta " << i << " " << thetas[i] << std::endl;
    if(thetas[i] != thetas[i])
      return std::vector<double>{0};
  }

  return thetas;
}


void FABRIK::drawDebug(QPainter * painter) {
  painter->drawText(0, 0, QString("Iterations: %1").arg(iterations));
  painter->drawRect(0, 20, 200, 100);
  double difPix = 200.0/diffs.size();
  double maxdif = 0;
  for (size_t i = 0; i < diffs.size(); i++) {
    maxdif = std::max(maxdif, diffs[i]);
  }
  double dDiff = 100.0 / maxdif;

  for (size_t i = 0; i < diffs.size(); i++) {
    painter->drawLine(i*difPix, 120, i*difPix, 120-diffs[i]*dDiff );
    if(i > 0) {
      painter->drawLine((i-1)*difPix, 120-diffs[i-1]*dDiff, i*difPix, 120-diffs[i]*dDiff );
    }
  }
  painter->drawText(0, 130, QString("max e: %1").arg(maxdif));

}

void FABRIK::drawCalculation(QPainter * painter, int iteration) {
  if(iteration > 0)
    for(size_t k = 0; k < jointPositionHistory.size(); k++) {
      for(size_t i = 0; i < jointPositionHistory[k].size(); i++) {
        if(k % 2 == 1)
          painter->drawEllipse(jointPositionHistory[k][i](0)-5, jointPositionHistory[k][i](1)-5, 10, 10);
        else
          painter->drawRect(jointPositionHistory[k][i](0)-5, jointPositionHistory[k][i](1)-5, 10, 10);
        if(i > 0)
          painter->drawLine(jointPositionHistory[k][i-1](0), jointPositionHistory[k][i-1](1), jointPositionHistory[k][i](0), jointPositionHistory[k][i](1));
      }
    }
}




Eigen::Vector3d FABRIK::calcJointPos(std::vector<double> thetas, std::vector<double> ls, int index) const {
  Eigen::Vector3d sumPos;
  sumPos << 0,0,0;
  double thetaSum = 0;

  int i = index;
  if(i == FABRIK::EndEffector) i = thetas.size();

  for(auto j = 0; j < i; j++) {
    thetaSum += thetas[j];
    sumPos(0) += ls[j] * std::cos(thetaSum * M_PI / 180);
    sumPos(1) += ls[j] * std::sin(thetaSum * M_PI / 180);
  }
  return sumPos;
}

std::vector<double> FABRIK::calcThetas(std::vector<Eigen::Vector3d> jointPositions) const {
  std::vector<double> thetas;

  thetas.push_back(std::atan2(jointPositions[1](1), jointPositions[1](0)) * 180 / M_PI);

  Eigen::Vector3d u;
  Eigen::Vector3d v;
  for(size_t i = 1; i < jointPositions.size()-1; i++) {
    u = jointPositions[i] - jointPositions[i-1];
    v = jointPositions[i+1] - jointPositions[i];

    thetas.push_back((std::atan2(v(1), v(0)) - std::atan2(u(1), u(0))) * 180 / M_PI);

  }

  return thetas;
}
