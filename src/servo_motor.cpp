#include "include/servo_motor.h"

using namespace chrono;

double ChFunctionPID::Get_y(double t) const {
  double desired_angle = motor_funct_->Get_y(t);
  double desired_angular_speed = motor_funct_->Get_y_dx(t);
  double curr_angle = engine_->Get_mot_rot();
  double curr_angular_speed = engine_->Get_mot_rot_dt();
  // cum_error_ += desired_angle - curr_angle;
  double torque = p_gain_ * (desired_angle - curr_angle) +
                  d_gain_ * (desired_angular_speed - curr_angular_speed) +
                  cum_error_ * i_gain_;
  torque = std::max(std::min(t_limit_, torque), -t_limit_);
  return torque;
}

void ChServoMotor::Initialize(std::shared_ptr<ChFunction> motor_funct,
                              ChLinkEngine::eCh_eng_mode mode_flag) {
  motor_mode_ = mode_flag;
  engine_->Set_eng_mode(mode_flag);
  if (mode_flag == ChLinkEngine::ENG_MODE_ROTATION) {
    engine_->Set_rot_funct(motor_funct);
  } else if (mode_flag == ChLinkEngine::ENG_MODE_TORQUE) {
    std::shared_ptr<ChFunctionPID> torque_funct(
        new ChFunctionPID(motor_funct, engine_));
    engine_->Set_tor_funct(torque_funct);
  }
}
