#include <Eigen/Dense>

#include "include/contact_reporter.h"
#include "include/controller.h"
#include "include/robot.h"
#include "include/servo_motor.h"
#include "include/vector_utility.h"

namespace {

double sigmoid(double sigma, double x) { return 1 / (1 + exp(-sigma * x)); }

double GetBodyXYPlaneAngle(chrono::ChBody *body_ptr) {
  double angle;
  chrono::ChVector<> axis;
  body_ptr->GetRot().Q_to_AngAxis(angle, axis);
  if (axis(2) < 0) {
    angle = -angle;
  }
  return angle;
}

} // namespace

using namespace chrono;

Controller::Controller(chrono::ChSystem *ch_system, Robot *i_robot)
    : ch_system_(ch_system), robot_(i_robot) {
  contact_reporter_ = new ContactExtractor(robot_->rigid_bodies.size());
  InitializeWaveParams();
}

void Controller::SetDefaultParams(const Json::Value &command) {
  wave_params_.amplitude = command.get("amplitude", 0.5).asDouble();
  wave_params_.frequency = command.get("num_waves", 1.5).asDouble() * CH_C_2PI *
                           wave_params_.wave_speed;
  wave_params_.head_phase = command.get("initial_phase", 0.0).asDouble();
  InitializeWaveParams();
}

void Controller::InitializeWaveParams() {
  const double kNumJoints = robot_->motors.size();

  wave_params_.theta.resize(kNumJoints);
  wave_params_.theta_dt.resize(kNumJoints);
  for (size_t i = 0; i < kNumJoints; ++i) {
    double phase = GetPhase(i);
    wave_params_.theta[i] = wave_params_.amplitude * sin(phase);
    wave_params_.theta_dt[i] =
        wave_params_.amplitude * wave_params_.frequency * cos(phase);
  }
}

void Controller::PropagateWave() {}

double Controller::GetPhase(size_t i) {
  const double kNumJoints = robot_->motors.size();
  double num_waves =
      wave_params_.frequency / (CH_C_2PI * wave_params_.wave_speed);
  return wave_params_.head_phase -
         double(kNumJoints - i - 1) * num_waves / kNumJoints * CH_C_2PI;
}

void Controller::Wrap() {
  // if (head_strategy_count_down_ == -50) {
  //   auto contact_indices = CharacterizeContacts();
  //   if (contact_indices.empty() || contact_indices.size() >= 3) {
  //     return;
  //   }
  //   head_index_ = contact_indices.back();
  //   if (head_index_ < 22 || head_index_ > 27) {
  //     return;
  //   }
  //   head_strategy_count_down_ = 100;
  // }
  //
  // if (head_strategy_count_down_ > 0) {
  //   std::cout << head_strategy_count_down_ << std::endl;
  //   for (int i = 0; i < robot_->motors.size(); ++i) {
  //     double desired_angle_dt = wave_params_.amplitudes[i] *
  //                               wave_params_.phases_dt[i] *
  //                               cos(wave_params_.phases[i]);
  //     wave_params_.theta_dt[i] -=
  //         5 * desired_angle_dt *
  //         exp(-(i - head_index_) * (i - head_index_) / 15.0);
  //   }
  // }
  // //
  // if (head_strategy_count_down_ > -50) {
  //   head_strategy_count_down_--;
  // }
}

void Controller::UpdateSnakeShape() {
  const size_t kNumJoints = robot_->motors.size();
  for (size_t i = 0; i < kNumJoints; ++i) {
    motor_functions_[i]->SetAngle(wave_params_.theta[i]);
    motor_functions_[i]->SetAngleDt(wave_params_.theta_dt[i]);
  }
}

void Controller::ExtractContactForces() {
  contact_reporter_->Reset();
  ch_system_->GetContactContainer()->ReportAllContacts(contact_reporter_);
  contact_forces_ = contact_reporter_->GetContactForces();
}

std::vector<size_t> Controller::CharacterizeContacts() {
  ExtractContactForces();
  std::vector<size_t> useful_contact_segs;
  // examine the contact force distribution along the body
  const double kNumSegs = robot_->rigid_bodies.size();
  // Ignore the first few head contact
  for (size_t i = 0; i < kNumSegs - 1; ++i) {
    if (contact_forces_[i].Length() < 1e-2) {
      // ignore this force;
      continue;
    }
    ChVector<> link_z =
        robot_->rigid_bodies[i]->TransformDirectionLocalToParent(
            ChVector<>(0, 0, 1));
    int contact_side = 0;
    if (dot(contact_forces_[i], link_z) < 0) {
      // contact on the right side
      contact_side = 1;
    } else {
      contact_side = -1;
    }

    // Now we can determine if this contact force is at the right phase
    double dth = motor_functions_[i]->Get_y_dx(0);
    // dth and contact_side should have different sign
    if (dth * contact_side < 0) {
      useful_contact_segs.push_back(i);
    }
  }
  return useful_contact_segs;
}

void Controller::Wiggle() {}

void Controller::Step(double dt) {
  // ProcessCommandQueue(dt);
  // Will not overflow.
  steps_++;
  auto contact_indices = CharacterizeContacts();
}

size_t Controller::GetNumMotors() { return robot_->motors.size(); }

void Controller::EnablePIDMotorControl() {
  InitializeWaveParams();
  motor_functions_.resize(0);
  for (size_t i = 0; i < robot_->motors.size(); ++i) {
    motor_functions_.emplace_back(
        new ChFunctionMotor(wave_params_.theta[i], 0));
    robot_->motors[i]->Initialize(motor_functions_[i],
                                  ChLinkEngine::ENG_MODE_TORQUE);
  }
}

void Controller::EnablePosMotorControl() {
  motor_functions_.resize(0);
  for (size_t i = 0; i < robot_->motors.size(); ++i) {
    motor_functions_.emplace_back(
        new ChFunctionMotor(wave_params_.theta[i], 0));
    robot_->motors[i]->Initialize(motor_functions_[i],
                                  ChLinkEngine::ENG_MODE_ROTATION);
  }
}
