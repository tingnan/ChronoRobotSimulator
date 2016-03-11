#ifndef INCLUDE_ROBOT_H_
#define INCLUDE_ROBOT_H_

#include <vector>

#include "json/json.h"
#include <Eigen/Core>
#include <core/ChMath.h>
#include <physics/ChBody.h>
#include <physics/ChLinkEngine.h>
#include <physics/ChSystem.h>

#include "include/servo_motor.h"

class RFTBody;

namespace Json {
class Value;
}

// The Snake robot
struct Robot {
  // Core components: rigid bodies and motors
  std::vector<std::shared_ptr<chrono::ChBody>> rigid_bodies;
  std::vector<std::shared_ptr<chrono::ChServoMotor>> motors;
  // To be used with RFT module
  std::vector<RFTBody> rft_bodies;
  // Cached value, to be used by the controller.
  std::vector<double> link_lengths;
  Eigen::MatrixXd inertia;
};

Robot BuildRobot(chrono::ChSystem *ch_system, const Json::Value &params);

#endif // INCLUDE_ROBOT_H_
