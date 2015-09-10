#ifndef INCLUDE_ROBOT_H_
#define INCLUDE_ROBOT_H_

#include <vector>

#include <core/ChMath.h>

class RFTBody;

namespace irr {
class ChIrrApp;
}

namespace chrono {
class ChBody;
class ChLinkLock;
}

namespace Json {
class Value;
}

class WorldBuilder {
public:
  WorldBuilder(class irr::ChIrrApp *app);
  void CreateRigidBodies(const Json::Value &params);
  std::vector<chrono::ChBody *> GetBodyList() { return body_list_; }

private:
  class irr::ChIrrApp *app_;
  std::vector<chrono::ChBody *> body_list_;
  std::vector<chrono::ChLinkLock *> engine_list_;
};

#endif // INCLUDE_ROBOT_H_
