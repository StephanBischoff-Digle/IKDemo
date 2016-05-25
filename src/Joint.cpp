#include "Joint.hpp"

#include <cmath>
#include <iostream>


// updates theta according to the joints speed
//  returns true if the joint reached it's target angle
bool Joint::update(int elapsedMS)
{
  if(elapsedMS < 0)
    return false;

  if(targetTheta == theta)
    return true;

  int direction = 1;
  if(targetTheta < theta)
    direction = -1;

  theta += direction * speed * elapsedMS / 1000.0;
  if((direction > 0 && theta > targetTheta) || (direction < 0 && theta < targetTheta))
    theta = targetTheta;

  return theta == targetTheta;
}

void Joint::setTargetTheta(double value) {
  double difference = value - targetTheta;
  while (difference < -180) difference += 360;
  while (difference > 180) difference -= 360;


  targetTheta += difference;
}
