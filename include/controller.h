#ifndef INCLUDE_CONTROLLER_H_
#define INCLUDE_CONTROLLER_H_

#include <vector>

namespace chrono {
class ChLinkLock;
} // namespace chrono

class RobotController {
public:
  RobotController();
  ~RobotController() {}
  void AddEngine(chrono::ChLinkLock *p) { motors_.push_back(p); }
  void EnablePositionControl();
  void EnableActiveLifting();
  void EnableTorqueControl();

private:
  std::vector<chrono::ChLinkLock *> motors_;
};

#endif // INCLUDE_CONTROLLER_H_
