#ifndef INCLUDE_CONTROLLER_H_
#define INCLUDE_CONTROLLER_H_

#include <vector>

namespace chrono {
class ChLinkLock;
class ChBody;
} // namespace chrono

class RobotController {
public:
  void SetControlStrategy();
  void AddEngine(chrono::ChLinkLock *p) { engine_list_.push_back(p); }
  void AddBody(chrono::ChBody *p) { body_list_.push_back(p); }

private:
  std::vector<chrono::ChBody *> body_list_;
  std::vector<chrono::ChLinkLock *> engine_list_;
};

#endif // INCLUDE_CONTROLLER_H_
