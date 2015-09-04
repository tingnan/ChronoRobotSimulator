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
}

namespace Json {
class Value;
}

class WorldBuilder {
public:
  WorldBuilder(class irr::ChIrrApp *app);
  void CreateRigidBodies(const Json::Value &body_list);
  std::vector<chrono::ChBody *> GetBodyList() { return ch_body_list_; }

private:
  class irr::ChIrrApp *app_;
  std::vector<chrono::ChBody *> ch_body_list_;
};

#endif // INCLUDE_ROBOT_H_
