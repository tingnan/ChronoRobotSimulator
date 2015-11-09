#include "include/chfunction_controller.h"
#include "include/controller.h"

using namespace chrono;
// The parameters for the function
double ChFunctionController::Get_y(double t) {
  double torque = ComputeDriveTorque(t) + 1.0 * GetMediaTorque(t);
  torque = std::max(std::min(torque_limit, torque), -torque_limit);
  return torque;
}

double ChFunctionController::GetMediaTorque(double t) {
  return controller_->GetMediaTorque(index_, t);
}

// The low level PID controller in motor.
double ChFunctionController::ComputeDriveTorque(double t) {
  double desired_angle = controller_->GetAngle(index_, t);
  double desired_angular_speed = controller_->GetAngularSpeed(index_, t);
  double curr_angle = controller_->GetEngine(index_)->Get_mot_rot();
  double curr_angular_speed = controller_->GetEngine(index_)->Get_mot_rot_dt();
  cum_error_ += desired_angle - curr_angle;
  double torque = p_gain * (desired_angle - curr_angle) +
                  d_gain * (desired_angular_speed - curr_angular_speed) +
                  cum_error_ * i_gain;
  return torque;
}
