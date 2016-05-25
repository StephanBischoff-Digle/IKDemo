#ifndef KINEMATICCHAIN_HPP
#define KINEMATICCHAIN_HPP

#include <vector>
#include <memory>
#include <utility>
#include <QPainter>

#include "Joint.hpp"

class InverseKinematic;

class KinematicChain {
public:
  std::shared_ptr<Joint> getJoint(int index) const {return chain[index];};
  bool updateChain(int elapsedMS);
  std::vector<std::shared_ptr<Joint>>::size_type size() const {return chain.size();}
  std::vector<double> getEffectorPosition() const;
  void addJoint(std::shared_ptr<Joint> joint) {chain.emplace_back(joint);}
  void removeLast() {chain.pop_back();}

  void setIK(std::shared_ptr<InverseKinematic> ik) {this->ik = ik;}
  std::shared_ptr<InverseKinematic> getIK() const {return ik;}

  void balanceSpeed();

private:
  std::vector<std::shared_ptr<Joint>> chain;
  std::shared_ptr<InverseKinematic> ik;

};

class InverseKinematic {
public:
  virtual std::vector<double> solveIK(std::shared_ptr<KinematicChain> chain, int targetX, int targetY) = 0;
  virtual void drawDebug(QPainter * painter) = 0;
  virtual void drawCalculation(QPainter * painter, int iteration) = 0;
  virtual int getSubclassIdentifier() const = 0;
};

#endif
