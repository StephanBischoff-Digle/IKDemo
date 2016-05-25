#include "AnalyticIK.hpp"

#include <cmath>
#include <iostream>

std::vector<double> AnalyticIK::solveIK(std::shared_ptr<KinematicChain> chain, int targetX, int targetY) {
  l1 = chain->getJoint(0)->getLinkLength();
  l2 = chain->getJoint(1)->getLinkLength();

  t21 = std::atan2(std::sqrt(1-std::pow((std::pow(targetX,2) + std::pow(targetY,2) - std::pow(l1,2) - std::pow(l2,2))/(2 * l1 * l2),2)), (std::pow(targetX,2) + std::pow(targetY,2) - std::pow(l1,2) - std::pow(l2,2))/(2 * l1 * l2));
  t11 = std::atan2(targetY, targetX) - std::atan2(l2 * std::sin(t21), l1 + l2 * std::cos(t21));

  t22 = std::atan2(-std::sqrt(1-std::pow((std::pow(targetX,2) + std::pow(targetY,2) - std::pow(l1,2) - std::pow(l2,2))/(2 * l1 * l2),2)), (std::pow(targetX,2) + std::pow(targetY,2) - std::pow(l1,2) - std::pow(l2,2))/(2 * l1 * l2));
  t12 = std::atan2(targetY, targetX) - std::atan2(l2 * std::sin(t22), l1 + l2 * std::cos(t22));

  t11 = t11 * 180 / M_PI;
  t21 = t21 * 180 / M_PI;
  t12 = t12 * 180 / M_PI;
  t22 = t22 * 180 / M_PI;

  if(t11==t11 && t21==t21) {
    // std::cerr << "\e[1;31mDEBUG:\e[0m " << "not NaN" << std::endl;
    return std::vector<double> {t11, t21};
  } else {
    return std::vector<double>();
  }

}

void AnalyticIK::drawDebug(QPainter * painter) {
  painter->drawRect(1,1,2,2);
}

void AnalyticIK::drawCalculation(QPainter * painter, int iteration) {
  if(iteration > 0) {
    painter->save();

    painter->rotate(t11);
    // painter->drawEllipse(-4, -4, 9, 9);
    painter->drawLine(0, 0, l1, 0);
    painter->translate(l1, 0);

    painter->rotate(t21);
    // painter->drawEllipse(-4, -4, 9, 9);
    painter->drawLine(0, 0, l2, 0);
    painter->translate(l2, 0);
    painter->restore();

    painter->save();

    painter->rotate(t12);
    // painter->drawEllipse(-4, -4, 9, 9);
    painter->drawLine(0, 0, l1, 0);
    painter->translate(l1, 0);

    painter->rotate(t22);
    // painter->drawEllipse(-4, -4, 9, 9);
    painter->drawLine(0, 0, l2, 0);
    painter->translate(l2, 0);
    painter->restore();
  }
}
