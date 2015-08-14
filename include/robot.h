#ifndef INCLUDE_ROBOT_H_
#define INCLUDE_ROBOT_H_

#include <vector>

#include <core/ChMath.h>

#include "include/controller.h"

class RFTBody;

namespace irr {
class ChIrrApp;
}

namespace chrono {
class ChBody;
}

class ChronoRobotBuilder {
public:
  ChronoRobotBuilder(class irr::ChIrrApp *app);
  void BuildRobot();
  void ResetRobot();
  class RobotController *GetController() { return &controller_; }
  std::vector<RFTBody> &getRFTBodyList() { return rft_body_list_; }
  chrono::ChVector<> GetRobotCoMPosition();
  void SetCollide(bool);

private:
  class irr::ChIrrApp *app_;
  std::vector<RFTBody> rft_body_list_;
  std::vector<chrono::ChBody *> ch_body_list_;
  RobotController controller_;
};

#endif // INCLUDE_ROBOT_H_
