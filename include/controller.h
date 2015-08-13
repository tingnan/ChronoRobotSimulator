#ifndef INCLUDE_CONTROLLER_H_
#define INCLUDE_CONTROLLER_H_

#include <vector>

namespace chrono {
class ChLinkEngine;
} // namespace chrono

class RobotController {
public:
  RobotController();
  ~RobotController() {}
  void AddEngine(chrono::ChLinkEngine *p) { motors_.push_back(p); }
  void SetControlSet(class SnakeControlSet *p) { robot_params_ = p; }
  void PositionControl();
  void ActiveLifting();
  void TorqueControl();

private:
  class SnakeControlSet *robot_params_;
  std::vector<chrono::ChLinkEngine *> motors_;
};

#endif // INCLUDE_CONTROLLER_H_
