#ifndef INCLUDE_CONTROLLER_H_
#define INCLUDE_CONTROLLER_H_

#include <vector>

namespace chrono {
class ChLinkEngine;
} // namespace chrono

class RobotController {
protected:
  class SnakeControlSet *mSnakeParams;
  std::vector<chrono::ChLinkEngine *> mEngines;

public:
  RobotController();
  ~RobotController() {}
  void AddEngine(chrono::ChLinkEngine *p) { mEngines.push_back(p); }
  void SetControlSet(class SnakeControlSet *p) { mSnakeParams = p; }
  void PositionControl();
  void ActiveLifting();
  void TorqueControl();
};

#endif // INCLUDE_CONTROLLER_H_
