#include "include/chfunction_controller.h"
#include "include/controller.h"

using namespace chrono;
// The parameters for the function
double ChFunctionController::Get_y(double t) {
  double torque =
      ComputeDriveTorque(t) + ComputeLimitTorque(t) + 1.0 * GetMediaTorque(t);
  /*
double torque_contact = GetContactTorque(t);
double desired_angular_speed = controller_->GetAngularSpeed(index_, t);
if (torque_contact * desired_angular_speed > 0) {
torque += 0.0 * torque_contact;
}
  */
  // torque += 1.0 * torque_contact;

  torque = std::max(std::min(torque_limit, torque), -torque_limit);
  return torque;
}

double ChFunctionController::GetContactTorque(double t) {
  return controller_->GetContactTorque(index_, t);
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
  // std::cout << index_ << " " << curr_angular_speed << std::endl;
  cum_error_ += desired_angle - curr_angle;
  double torque = p_gain * (desired_angle - curr_angle) +
                  d_gain * (desired_angular_speed - curr_angular_speed) +
                  cum_error_ * i_gain;
  // torque -= 1e-3 * curr_angular_speed;
  return torque;
}

double ChFunctionController::ComputeLimitTorque(double t) {
  auto engine = controller_->GetEngine(index_);
  double curr_angle = engine->Get_mot_rot();
  double angle_limit_compliance = 1;
  if (curr_angle > angle_limit) {
    return -angle_limit_compliance * (curr_angle - angle_limit);
  }
  if (curr_angle < -angle_limit) {
    return -angle_limit_compliance * (curr_angle - angle_limit);
  }
  return 0;
}
