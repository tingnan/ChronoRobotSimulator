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

// The Snake robot
struct Robot {
  std::vector<chrono::ChBody *> body_list;
  std::vector<double> body_length_list;
  std::vector<chrono::ChLinkEngine *> engine_list;
  std::vector<RFTBody> rft_body_list;
};

void BuildWorld(chrono::ChSystem *ch_system, const Json::Value &params);
Robot BuildRobot(chrono::ChSystem *ch_system, const Json::Value &params);

#endif // INCLUDE_ROBOT_H_
