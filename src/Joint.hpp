#ifndef JOINT_HPP
#define JOINT_HPP

// Represents a joint and it's direct armsegment (a.k.a. link)
class Joint {
public:
  // constructs an static point joint
  Joint()
    : speed(0)
    , maxSpeed(0)
    , theta(0)
    , targetTheta(0)
    , linkLength(0)
    {};

  // constructs a joint
  Joint(double theta, double linkLength, double maxSpeed)
    : speed(maxSpeed)
    , maxSpeed(maxSpeed)
    , theta(theta)
    , targetTheta(theta)
    , linkLength(linkLength)
    {};

  void setTargetTheta(double value);
  void setMaxSpeed(double value)    {maxSpeed = value; speed = value;}
  void setlinkLength(double value)  {linkLength = value;}
  void setSpeed(double value)       {speed = value;}

  double getSpeed()         const {return this->speed;}
  double getMaxSpeed()      const {return this->maxSpeed;}
  double getTheta()         const {return this->theta;}
  double getTargetTheta()   const {return this->targetTheta;}
  double getLinkLength()    const {return this->linkLength;}

  // updates the jointangle in accordance with its speed
  bool update(int elapsedMS);
private:
  double speed;
  double maxSpeed;
  double theta;
  double targetTheta;
  double linkLength;
};

#endif
