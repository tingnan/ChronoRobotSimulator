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

Controller::Controller(chrono::ChSystem *ch_system, Robot *i_robot,
                       double time_step)
    : ch_system_(ch_system), robot_(i_robot), time_step_(time_step) {
  contact_reporter_ = new ContactExtractor(robot_->rigid_bodies.size());
  InitializeWaveParams();
}

void Controller::SetDefaultParams(const Json::Value &command) {
  wave_params_.amplitude = command.get("amplitude", 0.5).asDouble();
  wave_params_.num_waves = command.get("num_waves", 1.5).asDouble();
  wave_params_.head_phase = command.get("initial_phase", 0.0).asDouble();
  InitializeWaveParams();
}

void Controller::InitializeWaveParams() {
  const double kNumJoints = robot_->motors.size();

  wave_params_.theta.resize(kNumJoints);
  wave_params_.theta_dt.resize(kNumJoints);
  wave_params_.theta_ref.resize(kNumJoints);
  for (size_t i = 0; i < kNumJoints; ++i) {
    double phase = GetPhase(i);
    wave_params_.theta[i] = wave_params_.amplitude * sin(phase);
    wave_params_.theta_ref[i] = wave_params_.theta[i];
    wave_params_.theta_dt[i] =
        wave_params_.amplitude * wave_params_.frequency * cos(phase);
  }
}

double Controller::GetPhase(size_t i) {
  const double kNumJoints = robot_->motors.size();
  return wave_params_.head_phase -
         double(kNumJoints - 1 - i) * wave_params_.num_waves / kNumJoints *
             CH_C_2PI;
}

void Controller::UpdateSnakeShape() {
  const size_t kNumJoints = robot_->motors.size();
  for (size_t i = 0; i < kNumJoints; ++i) {
    motor_functions_[i]->SetAngle(wave_params_.theta[i]);
  }
}

void Controller::ExtractContactForces() {
  contact_reporter_->Reset();
  ch_system_->GetContactContainer()->ReportAllContacts(contact_reporter_);
  contact_forces_ = contact_reporter_->GetContactForces();
}

void Controller::CharacterizeContacts() {
  ExtractContactForces();
  // examine the contact force distribution along the body
  const size_t kNumSegs = robot_->rigid_bodies.size();
  // Ignore the first few head contact
  for (size_t i = kNumSegs - 1; i != 0; --i) {
    if (contact_forces_[i].Length() < 1e-2) {
      // ignore this force;
      continue;
    }
    ChVector<> body_left_direction =
        robot_->rigid_bodies[i]->TransformDirectionLocalToParent(
            ChVector<>(0, 0, 1));
    int contact_side = 0;
    if (dot(contact_forces_[i], body_left_direction) < 0) {
      // contact on the right side
      contact_side = 1;
    } else {
      contact_side = -1;
    }

    // Now we can determine if this contact force is at the correct phase
    double dth = wave_params_.theta[i + 1] - wave_params_.theta[i];
    // dth and contact_side should have different sign
    if (dth * contact_side < 0) {
      contact_side_ = contact_side;
      contact_index_ = i;
      system_state_ = SnakeRobotState::wrap;
      wrap_count_down_ = 200;
      std::fill(wave_params_.theta_dt.begin(), wave_params_.theta_dt.end(), 0);
      const size_t min_index = fmax(contact_index_ - 3, 0);
      const size_t max_index = fmin(contact_index_ + 3, kNumSegs);
      for (size_t i = min_index; i < max_index; ++i) {
        wave_params_.theta_ref[i] = contact_side_ * chrono::CH_C_PI / 10.0;
        std::cout << wave_params_.theta_ref[i] << std::endl;
        wave_params_.theta_dt[i] =
            (wave_params_.theta_ref[i] - wave_params_.theta[i]) /
            wrap_count_down_;
      }
      break;
    }
  }
}

void Controller::PropagateWave() {
  wave_params_.head_phase = wave_params_.frequency * steps_ * time_step_;
  const size_t kNumSegs = robot_->rigid_bodies.size();
  size_t total_steps = wave_params_.num_waves * chrono::CH_C_2PI /
                       wave_params_.frequency / time_step_;
  size_t propagation_interval = double(total_steps) / kNumSegs;
  size_t rem_step = steps_ % propagation_interval;
  if (rem_step == 0) {
    // now do the shift back
    for (size_t i = 0; i < kNumSegs - 1; ++i) {
      wave_params_.theta_ref[i] = wave_params_.theta_ref[i + 1];
    }
    wave_params_.theta_ref[kNumSegs - 1] =
        wave_params_.amplitude * sin(GetPhase(kNumSegs - 1));
  }

  double rem_ratio = double(rem_step) / propagation_interval;
  for (size_t i = 0; i < kNumSegs - 1; ++i) {
    wave_params_.theta[i] =
        (wave_params_.theta_ref[i + 1] - wave_params_.theta_ref[i]) *
            rem_ratio +
        wave_params_.theta_ref[i];
  }
  wave_params_.theta[kNumSegs - 1] =
      rem_ratio * (wave_params_.amplitude * sin(GetPhase(kNumSegs - 1)) -
                   wave_params_.theta_ref[kNumSegs - 1]) *
          rem_ratio +
      wave_params_.theta_ref[kNumSegs - 1];
}

void Controller::Wrap() {
  const size_t kNumSegs = robot_->rigid_bodies.size();
  for (size_t i = 0; i < kNumSegs; ++i) {
    wave_params_.theta[i] += wave_params_.theta_dt[i];
  }

  wrap_count_down_--;
  if (wrap_count_down_ <= 0) {
    system_state_ = SnakeRobotState::offset;
  }
}

void Controller::Wiggle() {
  steps_++;
  PropagateWave();
}

void Controller::Step() {

  // will change the state of the system;

  switch (system_state_) {
  case SnakeRobotState::wiggle:
    CharacterizeContacts();
    Wiggle();
    break;
  case SnakeRobotState::wrap:
    Wrap();
    break;
  default:
    Wiggle();
    break;
  }
  UpdateSnakeShape();
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
