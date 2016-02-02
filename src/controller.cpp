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

Eigen::MatrixXd ChainJacobianDx(const std::vector<double> &link_lengths,
                                const Eigen::VectorXd &sin_thetas) {
  const size_t kNumSegs = link_lengths.size();
  Eigen::MatrixXd jacobian_dx(kNumSegs, kNumSegs + 2);
  jacobian_dx.setZero();
  for (size_t i = 0; i < kNumSegs; ++i) {
    double l = link_lengths[i];
    jacobian_dx(i, i) = -0.5 * l * sin_thetas(i);
    jacobian_dx(i, kNumSegs) = 1;
    for (int j = i - 1; j >= 0; --j) {
      double l = link_lengths[j];
      jacobian_dx(i, j) = jacobian_dx(i, j + 1) - l * sin_thetas(j);
    }
  }
  return jacobian_dx;
}

Eigen::MatrixXd ChainJacobianDy(const std::vector<double> &link_lengths,
                                const Eigen::VectorXd &cos_thetas) {
  const size_t kNumSegs = link_lengths.size();
  Eigen::MatrixXd jacobian_dy(kNumSegs, kNumSegs + 2);
  jacobian_dy.setZero();
  for (size_t i = 0; i < kNumSegs; ++i) {
    double l = link_lengths[i];
    jacobian_dy(i, i) = 0.5 * l * cos_thetas(i);
    jacobian_dy(i, kNumSegs + 1) = 1;
    for (int j = i - 1; j >= 0; --j) {
      double l = link_lengths[j];
      jacobian_dy(i, j) = jacobian_dy(i, j + 1) + l * cos_thetas(j);
    }
  }
  return jacobian_dy;
}

Eigen::MatrixXd ChainJacobianDw(const std::vector<double> &link_lengths) {
  const size_t kNumSegs = link_lengths.size();
  Eigen::MatrixXd jacobian_dw(kNumSegs, kNumSegs + 2);
  jacobian_dw.setZero();
  for (size_t i = 0; i < kNumSegs; ++i) {
    for (size_t j = 0; j <= i; ++j) {
      jacobian_dw(i, j) = 1;
    }
  }
  return jacobian_dw;
}

Eigen::MatrixXd ComputeChainJacobianTailFrame(
    const std::vector<chrono::ChSharedPtr<chrono::ChBody>> &rigid_bodies,
    const std::vector<double> &link_lengths) {
  const size_t kNumSegs = rigid_bodies.size();
  Eigen::VectorXd thetas(kNumSegs);
  thetas.setZero();
  for (size_t i = 0; i < kNumSegs; ++i) {
    thetas(i) = GetBodyXYPlaneAngle(rigid_bodies[i].get());
  }

  Eigen::VectorXd sin_thetas(kNumSegs);
  Eigen::VectorXd cos_thetas(kNumSegs);
  for (size_t j = 0; j < kNumSegs; ++j) {
    sin_thetas(j) = sin(thetas(j));
    cos_thetas(j) = cos(thetas(j));
  }
  auto jacobian_dx = ChainJacobianDx(link_lengths, sin_thetas);
  auto jacobian_dy = ChainJacobianDy(link_lengths, cos_thetas);
  auto jacobian_dw = ChainJacobianDw(link_lengths);

  Eigen::MatrixXd jacobian(3 * kNumSegs, kNumSegs + 2);
  // Fill the elements (partial x_i, y_i, theta_i / partial q_j)
  for (size_t i = 0; i < kNumSegs; ++i) {
    for (size_t j = 0; j < kNumSegs + 2; ++j) {
      jacobian(3 * i + 0, j) = jacobian_dx(i, j);
      jacobian(3 * i + 1, j) = jacobian_dy(i, j);
      jacobian(3 * i + 2, j) = jacobian_dw(i, j);
    }
  }
  // std::cout << jacobian << std::endl;
  return jacobian;
}

Eigen::MatrixXd ChainJacobianCoMTransform(const Eigen::MatrixXd &jacobian_dx,
                                          const Eigen::MatrixXd &jacobian_dy) {
  const size_t kNumSegs = jacobian_dx.rows();
  Eigen::MatrixXd jacobian_th(kNumSegs, kNumSegs);
  jacobian_th.setIdentity();
  for (size_t j = 1; j < kNumSegs; ++j) {
    jacobian_th(0, j) = -1 + double(j) / kNumSegs;
  }
  // std::cout << jacobian_th << std::endl;

  Eigen::VectorXd jacobian_dx_row_mean(kNumSegs);
  jacobian_dx_row_mean.setZero();
  Eigen::VectorXd jacobian_dy_row_mean(kNumSegs);
  jacobian_dy_row_mean.setZero();
  for (size_t j = 0; j < kNumSegs; ++j) {
    jacobian_dx_row_mean(j) = -jacobian_dx.col(j).sum() / kNumSegs;
    jacobian_dy_row_mean(j) = -jacobian_dy.col(j).sum() / kNumSegs;
  }
  jacobian_dx_row_mean = jacobian_dx_row_mean.transpose() * jacobian_th;
  jacobian_dy_row_mean = jacobian_dy_row_mean.transpose() * jacobian_th;
  // std::cout << jacobian_dx_row_mean.transpose() << std::endl;
  // std::cout << jacobian_dy_row_mean.transpose() << std::endl;

  Eigen::MatrixXd jj(kNumSegs + 2, kNumSegs + 2);
  jj.setZero();
  jj.block(0, 0, kNumSegs, kNumSegs) = jacobian_th;
  jj.block(kNumSegs + 0, 0, 1, kNumSegs) = jacobian_dx_row_mean.transpose();
  jj.block(kNumSegs + 1, 0, 1, kNumSegs) = jacobian_dy_row_mean.transpose();
  jj(kNumSegs, kNumSegs) = 1;
  jj(kNumSegs + 1, kNumSegs + 1) = 1;
  return jj;
}

Eigen::MatrixXd ComputeChainJacobianCoMFrame(
    const std::vector<chrono::ChSharedPtr<chrono::ChBody>> &rigid_bodies,
    const std::vector<double> &link_lengths) {
  const size_t kNumSegs = rigid_bodies.size();
  Eigen::VectorXd thetas(kNumSegs);
  thetas.setZero();
  for (size_t i = 0; i < kNumSegs; ++i) {
    thetas(i) = GetBodyXYPlaneAngle(rigid_bodies[i].get());
  }

  Eigen::VectorXd sin_thetas(kNumSegs);
  Eigen::VectorXd cos_thetas(kNumSegs);
  for (size_t j = 0; j < kNumSegs; ++j) {
    sin_thetas(j) = sin(thetas(j));
    cos_thetas(j) = cos(thetas(j));
  }
  auto jacobian_dx = ChainJacobianDx(link_lengths, sin_thetas);
  auto jacobian_dy = ChainJacobianDy(link_lengths, cos_thetas);
  auto jacobian_dw = ChainJacobianDw(link_lengths);

  auto jj = ChainJacobianCoMTransform(jacobian_dx, jacobian_dy);
  jacobian_dx = jacobian_dx * jj;
  jacobian_dy = jacobian_dy * jj;
  jacobian_dw = jacobian_dw * jj;

  Eigen::MatrixXd jacobian(3 * kNumSegs, kNumSegs + 2);
  jacobian.setZero();
  // Fill the elements (partial x_i, y_i / partial theta_j)
  for (size_t i = 0; i < kNumSegs; ++i) {
    for (size_t j = 0; j < kNumSegs + 2; ++j) {
      jacobian(3 * i + 0, j) = jacobian_dx(i, j);
      jacobian(3 * i + 1, j) = jacobian_dy(i, j);
      jacobian(3 * i + 2, j) = jacobian_dw(i, j);
    }
  }
  // std::cout << jacobian << std::endl;
  return jacobian;
}

Eigen::VectorXd SolveChainInternalTorque(const Eigen::MatrixXd &inertia,
                                         const Eigen::MatrixXd &jacobian,
                                         const Eigen::VectorXd &f_ext) {

  Eigen::MatrixXd inertia_q = jacobian.transpose() * inertia * jacobian;
  Eigen::VectorXd generalized_force = jacobian.transpose() * f_ext;
  Eigen::Matrix3d inertia_com;
  inertia_com(0, 0) = inertia_q(0, 0);
  const size_t kSegs = inertia_q.rows() - 2;
  inertia_com.block<1, 2>(0, 1) = inertia_q.block<1, 2>(0, kSegs);
  inertia_com.block<2, 1>(1, 0) = inertia_q.block<2, 1>(kSegs, 0);
  inertia_com.block<2, 2>(1, 1) = inertia_q.block<2, 2>(kSegs, kSegs);
  Eigen::Vector3d generalized_force_com;
  generalized_force_com << generalized_force(0), generalized_force(kSegs),
      generalized_force(kSegs + 1);
  Eigen::Vector3d com_motion = inertia_com.ldlt().solve(generalized_force_com);
  Eigen::MatrixXd reduced_mat(inertia_q.rows(), 3);
  reduced_mat.col(0) = inertia_q.col(0);
  reduced_mat.col(1) = inertia_q.col(kSegs);
  reduced_mat.col(2) = inertia_q.col(kSegs + 1);
  Eigen::VectorXd interal_torques =
      reduced_mat * com_motion - generalized_force;
  // std::cout << interal_torques.transpose() << std::endl;
  return interal_torques;
}

void ShiftArray(Eigen::VectorXd &array, double default_value, bool forward) {
  // if shift from beginning to end
  size_t kArrayLength = array.rows();
  if (forward) {
    for (size_t i = 0; i < kArrayLength - 1; ++i) {
      array(i + 1) = array(i);
    }
    array(0) = default_value;
  } else {
    for (size_t i = 0; i < kArrayLength - 1; ++i) {
      array(i) = array(i + 1);
    }
    array(kArrayLength - 1) = default_value;
  }
}

} // namespace

using namespace chrono;

Controller::Controller(chrono::ChSystem *ch_system, Robot *i_robot)
    : ch_system_(ch_system), robot_(i_robot) {
  contact_reporter_ = new ContactExtractor(robot_->rigid_bodies.size());
  InitializeWindows();
}

void Controller::SetDefaultParams(const Json::Value &command) {
  num_waves_ = 0.5 / command.get("duration", 0.25).asDouble();
  default_amplitude_ = command.get("amplitude", 0.5).asDouble();
  InitializeWindows();
}

void Controller::ProcessCommandQueue(double dt) {
  // group_velocity_ = 0.05;
  // const double kPropagationCycle = (1. / group_velocity_);
  // auto command = command_queue_.front();
  // double duration =
  //     kPropagationCycle * command.get("duration", 0.25).asDouble();
  // command_frequency_ = CH_C_PI / duration;
  // command_amplitude_ = command.get("amplitude", 0.5).asDouble();
}

Eigen::VectorXd Controller::ComputeInternalTorque() {
  const size_t kNumSegs = robot_->rigid_bodies.size();
  const size_t kNJacDim = 3;
  Eigen::VectorXd forces_contact(kNJacDim * kNumSegs);
  contact_reporter_->Reset();
  ch_system_->GetContactContainer()->ReportAllContacts2(contact_reporter_);
  auto reported_forces = contact_reporter_->GetContactForces();
  const size_t kXIdx = 0;
  const size_t kZIdx = 2;
  for (size_t i = 0; i < kNumSegs; ++i) {
    forces_contact(kNJacDim * i + 0) = reported_forces[i](kXIdx);
    forces_contact(kNJacDim * i + 1) = reported_forces[i](kZIdx);
    forces_contact(kNJacDim * i + 2) = 0;
  }
  // optionally, compute the forces from the media
  Eigen::VectorXd forces_media(kNJacDim * kNumSegs);
  for (size_t i = 0; i < kNumSegs; ++i) {
    auto rft_force = robot_->rigid_bodies[i]->Get_accumulated_force();
    forces_media(kNJacDim * i + 0) = rft_force(kXIdx);
    forces_media(kNJacDim * i + 1) = rft_force(kZIdx);
    forces_media(kNJacDim * i + 2) = 0;
  }
  auto forces = 0.0 * forces_media + forces_contact;

  auto jacobian =
      ComputeChainJacobianCoMFrame(robot_->rigid_bodies, robot_->link_lengths);
  auto torque = SolveChainInternalTorque(robot_->inertia, jacobian, forces);
  const size_t kNumJoints = robot_->motors.size();
  // subblock corresponding to each joint
  auto torque_int = torque.block(1, 0, kNumJoints, 1);
  // std::cout << torque_int.transpose() << std::endl;
  return torque_int;
}

WaveWindow Controller::GenerateDefaultWindow() {
  WaveWindow default_window;
  default_window.amp_modifier = 1;
  default_window.amplitude = default_amplitude_;
  default_window.frequency = default_frequency_;
  // Start at head
  const size_t kNumJoints = robot_->motors.size();
  default_window.window_start = kNumJoints - 1;
  default_window.window_width =
      std::max(ceil(kNumJoints / num_waves_ / 2), 1.0);
  return default_window;
}

void Controller::InitializeWindows() {
  // Initialize all wave windows using the default params
  wave_windows_.resize(0);
  wave_windows_.emplace_front(GenerateDefaultWindow());
  auto last_window = wave_windows_.begin();
  while (last_window->window_start >= 0) {
    wave_windows_.emplace_front(GenerateDefaultWindow());
    wave_windows_.front().window_start =
        last_window->window_start - wave_windows_.front().window_width;
    last_window = wave_windows_.begin();
  }
}

void PrintAllWindows(const std::list<WaveWindow> &windows) {
  for (auto &window : windows) {
    std::cout << window.window_start << ":"
              << window.window_start + window.window_width - 1 << ",";
  }
  std::cout << std::endl;
}

void Controller::PropagateWindows(double dt) {
  // The function move windows backwards
  const size_t kNumJoints = robot_->motors.size();
  const size_t kPropagationInterval = (1. / group_velocity_) / kNumJoints / dt;
  if (steps_ % kPropagationInterval == 0) {
    // We need at least 1 window
    assert(wave_windows_.size() > 0);
    // iterate through windows and shift the index
    for (auto &window : wave_windows_) {
      window.window_start--;
    }
    // Check the first window in the list for coverage. If a window moves out of
    // the snake body (s + w - 1 < 0), we recycle it
    auto &front_window = wave_windows_.front();
    if (front_window.window_start + int(front_window.window_width) - 1 < 0) {
      // past_windows_.emplace_back(front_window);
      wave_windows_.pop_front();
    }
    // Now check for coverage: is it necessary to append another window to the
    // back of the list? Notice that the list may have been changed!
    if (wave_windows_.size() == 0) {
      // This happens when there is only 1 joint along the body
      wave_windows_.emplace_back(GenerateDefaultWindow());
    }
    // back() function is undefined if the list is empty
    if (wave_windows_.back().window_start + wave_windows_.back().window_width <
        kNumJoints) {
      wave_windows_.emplace_back(GenerateDefaultWindow());
    }
    // PrintAllWindows(wave_windows_);
  }
}

void Controller::UpdateWindowParams(double dt) {
  // Compute the contact induced torques at each joint
  auto torque_int = ComputeInternalTorque();
  const size_t kNumJoints = robot_->motors.size();
  for (auto &window : wave_windows_) {
    // Compute the range of motors the window contains
    int beg = std::max(window.window_start, 0);
    int end =
        std::min(window.window_start + window.window_width, int(kNumJoints));
  }
}

void Controller::ApplyWindowParams() {
  const size_t kNumJoints = robot_->motors.size();
  for (auto &window : wave_windows_) {
    // Compute the range of motors the window contains
    int beg = std::max(window.window_start, 0);
    int end =
        std::min(window.window_start + window.window_width, int(kNumJoints));
    for (size_t i = beg; i < end; ++i) {
      // Apply the window params to the joint motor functions.
      motor_functions_[i]->SetAmplitude(window.amplitude * window.amp_modifier);
      motor_functions_[i]->SetFrequency(window.frequency);
    }
  }
}

void Controller::Step(double dt) {
  // ProcessCommandQueue(dt);
  // Will not overflow.
  steps_++;
  // For the simulation duration if won't overflow.
  for (auto &motor_function : motor_functions_) {
    motor_function->Step(dt);
  }

  // Update all window params according to the compliance parameter
  UpdateWindowParams(dt);
  // Apply the window params.
  ApplyWindowParams();
  // Propagate the windows from head to tail
  PropagateWindows(dt);
}

size_t Controller::GetNumMotors() { return robot_->motors.size(); }

void Controller::EnablePIDMotorControl() {
  motor_functions_.resize(0);
  auto &motors = robot_->motors;
  for (size_t i = 0; i < motors.size(); ++i) {
    motor_functions_.emplace_back(
        new ChFunctionMotor(default_amplitude_, default_frequency_,
                            double(i * num_waves_) / motors.size() * CH_C_2PI));
    motors[i]->Initialize(motor_functions_[i], ChLinkEngine::ENG_MODE_TORQUE);
  }
}

void Controller::EnablePosMotorControl() {
  motor_functions_.resize(0);
  auto &motors = robot_->motors;
  for (size_t i = 0; i < motors.size(); ++i) {
    motor_functions_.emplace_back(
        new ChFunctionMotor(default_amplitude_, default_frequency_,
                            double(i * num_waves_) / motors.size() * CH_C_2PI));
    motors[i]->Initialize(motor_functions_[i], ChLinkEngine::ENG_MODE_ROTATION);
  }
}
