#ifndef INCLUDE_ROBOT_H_
#define INCLUDE_ROBOT_H_

#include <vector>

#include <core/ChMath.h>

class RFTBody;
class SnakeControlSet {
public:
  // shape params
  double k;
  double A;
  // frequency
  double w;
  double h;
};

namespace irr {
class ChIrrApp;
}

namespace chrono {
class ChBody;
}

class ChronoRobotBuilder {
  // needs to be provided at initialization
  class irr::ChIrrApp *mApp;
  // needed at the building robot
  SnakeControlSet *mSnakeParams;
  // maybe needed
  std::vector<RFTBody> mRFTBodylist;
  std::vector<chrono::ChBody *> mCollisionObjs;
  class RFTSystem *mRFT;
  class RobotController *mController;

public:
  ChronoRobotBuilder(class irr::ChIrrApp *);
  void BuildRobot();
  void ResetRobot();
  void SetControlSet(SnakeControlSet *snakeParams) {
    mSnakeParams = snakeParams;
  }
  void SetRFTSystems(class RFTSystem *rft) { mRFT = rft; }
  class RobotController *GetController() { return mController; }
  std::vector<RFTBody> &getRFTBodyList() { return mRFTBodylist; }
  chrono::ChVector<> GetRobotCoMPosition();
  void BuildBoard(double);
  void SetCollide(bool);
};

#endif // INCLUDE_ROBOT_H_
