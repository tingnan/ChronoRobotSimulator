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
class ChLinkEngine;
}

namespace Json {
class Value;
}

struct Robot {
  std::vector<chrono::ChBody *> body_list;
  std::vector<chrono::ChLinkEngine *> engine_list;
  std::vector<RFTBody> rft_body_list;
};

Robot BuildRobotAndWorld(irr::ChIrrApp *ch_app, const Json::Value &params);

#endif // INCLUDE_ROBOT_H_
