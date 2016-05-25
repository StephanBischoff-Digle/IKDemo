#include "KinematicChain.hpp"

#include <cmath>
#include <iostream>

std::vector<double> KinematicChain::getEffectorPosition() const {
  std::vector<double> position(2, 0);
  double thetasSum = 0;
  for(auto j : chain) {
    thetasSum += j->getTheta();
    position[0] += j->getLinkLength() * std::cos(thetasSum * M_PI / 180);
    position[1] += j->getLinkLength() * std::sin(thetasSum * M_PI / 180);
  }

  return position;
}


bool KinematicChain::updateChain(int elapsedMS) {
  std::vector<bool> tmp;
  bool result = true;
  for(typeof(chain.size()) i = 0; i < chain.size(); i++) {
    tmp.push_back(chain[i]->update(elapsedMS));
  }

  for(auto b : tmp) {
    if(!b) {
      result = false;
      break;
    }
  }
  return result;
}

void KinematicChain::balanceSpeed() {
    std::vector<double> etas;
    std::vector<double> dts;
    double maxETA = 0;

    for (size_t i = 0; i < chain.size(); i++) {
      dts.push_back(std::abs(chain[i]->getTargetTheta() - chain[i]->getTheta()));
      etas.push_back(dts[i] / chain[i]->getMaxSpeed());
      if(maxETA < etas[i]) maxETA = etas[i];
    }

    for (size_t i = 0; i < chain.size(); i++) {
      chain[i]->setSpeed(dts[i] / maxETA);
    }
}
