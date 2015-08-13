#ifndef INCLUDE_ROBOT_H_
#define INCLUDE_ROBOT_H_

#include <vector>

#include <core/ChMath.h>

class RFTBody;
struct SnakeControlSet {
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
public:
  ChronoRobotBuilder(class irr::ChIrrApp *app);
  void BuildRobot();
  void ResetRobot();
  void SetControlSet(SnakeControlSet *snakeParams) {
    robot_params_ = snakeParams;
  }
  void SetRFTSystems(class RFTSystem *rft) { rft_system_ = rft; }
  class RobotController *GetController() { return controller_; }
  std::vector<RFTBody> &getRFTBodyList() { return body_list_; }
  chrono::ChVector<> GetRobotCoMPosition();
  void BuildBoard(double);
  void SetCollide(bool);

private:
  // needs to be provided at initialization
  class irr::ChIrrApp *app_;
  // needed at the building robot
  SnakeControlSet *robot_params_;
  // maybe needed
  std::vector<RFTBody> body_list_;
  std::vector<chrono::ChBody *> ch_body_list_;
  class RFTSystem *rft_system_;
  class RobotController *controller_;
};

#endif // INCLUDE_ROBOT_H_
