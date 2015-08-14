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
  void EnablePositionControl();
  void EnableActiveLifting();
  void EnableTorqueControl();

private:
  std::vector<chrono::ChLinkEngine *> motors_;
};

#endif // INCLUDE_CONTROLLER_H_
