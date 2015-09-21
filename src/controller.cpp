#include "include/controller.h"
#include "include/robot.h"

using namespace chrono;

void Controller::Step(double dt) {
  auto jacobian = ComputeJacobian(robot_);
  const size_t kNumSegs = robot_->body_list.size();
  auto gravity = ch_system_->Get_G_acc().Length();
  ChVectorDynamic<> gravity_force(3 * kNumSegs);
  for (size_t i = 0; i < kNumSegs; ++i) {
    gravity_force(3 *i + 0) = 0;
    gravity_force(3 *i + 1) = robot_->body_list[i]->GetMass() * gravity;
    // Torque
    gravity_force(3 *i + 2) = 0;
  }
  torques_ext_ = jacobian * gravity_force;
  // std::cout << torques_ext_(0) << " " << torques_ext_(1) << std::endl;
}

size_t Controller::GetNumEngines() { return robot_->engine_list.size(); }
ChLinkEngine *Controller::GetEngine(size_t i) { return robot_->engine_list[i]; }

double Controller::GetExtTorque(size_t i, double t) { return torques_ext_(i); }

double Controller::GetPatternAngle(size_t index, double t) {
  const double phase =
      double(index * 2) / robot_->engine_list.size() * CH_C_2PI;
  double desired_angle = amplitude_ * sin(omega_ * t + phase);
  return desired_angle;
}

double Controller::GetPatternAngularSpeed(size_t index, double t) {
  const double phase =
      double(index * 2) / robot_->engine_list.size() * CH_C_2PI;
  double desired_angular_speed = amplitude_ * omega_ * cos(omega_ * t + phase);
  return desired_angular_speed;
}
