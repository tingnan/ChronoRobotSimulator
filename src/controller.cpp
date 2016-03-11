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
    const std::vector<std::shared_ptr<chrono::ChBody>> &rigid_bodies,
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
    const std::vector<std::shared_ptr<chrono::ChBody>> &rigid_bodies,
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

void ShiftArray(std::vector<double> array, double default_value, bool forward) {
  // if shift from beginning to end
  size_t kArrayLength = array.size();
  if (forward) {
    for (size_t i = 0; i < kArrayLength - 1; ++i) {
      array[i + 1] = array[i];
    }
    array[0] = default_value;
  } else {
    for (size_t i = 0; i < kArrayLength - 1; ++i) {
      array[i] = array[i + 1];
    }
    array[kArrayLength - 1] = default_value;
  }
}

} // namespace

using namespace chrono;

Controller::Controller(chrono::ChSystem *ch_system, Robot *i_robot)
    : ch_system_(ch_system), robot_(i_robot) {
  contact_reporter_ = new ContactExtractor(robot_->rigid_bodies.size());
  InitializeWaveParams();
}

void Controller::SetDefaultParams(const Json::Value &command) {
  wave_params_.desired_amplitude = command.get("amplitude", 0.5).asDouble();
  wave_params_.frequency = command.get("num_waves", 1.5).asDouble() * 2 *
                           CH_C_2PI * wave_params_.wave_speed;
  wave_params_.initial_phase = command.get("initial_phase", 0.0).asDouble();
  InitializeWaveParams();
}

void Controller::InitializeWaveParams() {
  const double kNumJoints = robot_->motors.size();
  wave_params_.amplitudes.resize(kNumJoints);
  for (auto &amp : wave_params_.amplitudes) {
    amp = wave_params_.desired_amplitude;
  }
  wave_params_.amplitudes_dt.resize(kNumJoints);
  for (auto &amp_dt : wave_params_.amplitudes_dt) {
    amp_dt = 0;
  }
  wave_params_.desired_phases.resize(kNumJoints);
  double num_waves =
      wave_params_.frequency / (2 * CH_C_2PI * wave_params_.wave_speed);
  for (int i = kNumJoints - 1; i >= 0; --i) {
    // set a phase lag from head to tail
    wave_params_.desired_phases[i] =
        wave_params_.initial_phase -
        double(kNumJoints - i - 1) * num_waves / kNumJoints * CH_C_2PI;
  }
  // Just copy
  wave_params_.phases = wave_params_.desired_phases;
  wave_params_.phases_dt.resize(kNumJoints);
  for (auto &ph_dt : wave_params_.phases_dt) {
    ph_dt = wave_params_.frequency;
  }

  wave_params_.theta.resize(kNumJoints);
  wave_params_.theta_dt.resize(kNumJoints);
  for (size_t i = 0; i < kNumJoints; ++i) {
    wave_params_.theta[i] =
        wave_params_.amplitudes[i] * sin(wave_params_.phases[i]);
    wave_params_.theta_dt[i] = wave_params_.amplitudes[i] *
                               wave_params_.phases_dt[i] *
                               cos(wave_params_.phases[i]);
  }
}

Eigen::VectorXd Controller::ComputeInternalTorque() {
  const size_t kNumSegs = robot_->rigid_bodies.size();
  const size_t kNJacDim = 3;
  Eigen::VectorXd forces_contact(kNJacDim * kNumSegs);
  contact_reporter_->Reset();
  ch_system_->GetContactContainer()->ReportAllContacts(contact_reporter_);
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

// void PrintAllWindows(const std::list<WaveWindow> &windows) {
//   for (auto &window : windows) {
//     std::cout << window.window_start << ":"
//               << window.window_start + window.window_width - 1 << ":"
//               << window.amp_modifiers[0] << ",";
//   }
//   std::cout << std::endl;
// }

void Controller::PropagateWaveParams(double dt) {
  // // The function move windows backwards
  // const size_t kNumJoints = robot_->motors.size();
  // const size_t kPropagationInterval = (1. / group_velocity_) / kNumJoints /
  // dt;
  // if (steps_ % kPropagationInterval == 0) {
  //   // We need at least 1 window
  //   assert(wave_windows_.size() > 0);
  //   // iterate through windows and shift the index
  //   for (auto &window : wave_windows_) {
  //     window.window_start--;
  //   }
  //   // Check the first window in the list for coverage. If a window moves out
  //   of
  //   // the snake body (s + w - 1 < 0), we recycle it
  //   auto &front_window = wave_windows_.front();
  //   if (front_window.window_start + int(front_window.window_width) - 1 < 0) {
  //     // past_windows_.emplace_back(front_window);
  //     wave_windows_.pop_front();
  //   }
  //
  //   // Now check for coverage: is it necessary to append another window to
  //   the
  //   // back of the list? Notice that the list may have been changed!
  //   if (wave_windows_.size() == 0) {
  //     // This happens when there is only 1 joint along the body
  //     wave_windows_.emplace_back(GenerateDefaultWindow());
  //   }
  //   // back() function is undefined if the list is empty
  //   if (wave_windows_.back().window_start + wave_windows_.back().window_width
  //   <
  //       kNumJoints) {
  //     wave_windows_.emplace_back(GenerateDefaultWindow());
  //   }
  //   PrintAllWindows(wave_windows_);
  // }
}

void Controller::UpdateAmplitudes(double dt) {
  // Compute the contact induced torques at each joint
  auto torque_int = ComputeInternalTorque();
  const size_t kNumJoints = robot_->motors.size();
  // int contact_index = DetermineContactPosition();
  // for (auto &window : wave_windows_) {
  //   // Compute the range of motors the window contains
  //   int beg = std::max(window.window_start, 0);
  //   int end =
  //       std::min(window.window_start + window.window_width, int(kNumJoints));
  //   double shape_force = 0;
  //   for (size_t i = beg; i < end; ++i) {
  //     // The jacobian mapping
  //     shape_force -= robot_->motors[i]->GetMotorRotation() * torque_int(i);
  //   }
  //   for (size_t i = 0; i < window.amp_modifiers.size(); ++i) {
  //     int curr_motor_id = i + beg;
  //     if (curr_motor_id >= end) {
  //       continue;
  //     }
  //     double jnt_shape_force = shape_force;
  //     // Clamp the force
  //     const double kMaxShapeForce = 20.0;
  //     jnt_shape_force =
  //         std::max(std::min(jnt_shape_force, kMaxShapeForce),
  //         -kMaxShapeForce);
  //     // save the load history into its history queue
  //     // std::cout << jnt_shape_force << "\n";
  //     const double kDmp = 2.5;
  //     const double kSpr = 2.5;
  //     const double kMas = 1.0;
  //     double amp_modifier_ddt =
  //         (jnt_shape_force - window.amp_modifiers_dt[i] * kDmp -
  //          kSpr * (window.amp_modifiers[i] - window.amplitude)) /
  //         kMas;
  //     window.amp_modifiers_dt[i] += amp_modifier_ddt * dt;
  //     window.amp_modifiers[i] += window.amp_modifiers_dt[i] * dt;
  //     const double kMaxRelCurvature = 12;
  //     double max_amp = kMaxRelCurvature / window.window_width / 2;
  //     window.amp_modifiers[i] =
  //         std::max(std::min(window.amp_modifiers[i], max_amp), 0.0);
  //   }
  // }
}

void Controller::ApplyHeadStrategy() {
  if (head_strategy_count_down_ == -50) {
    auto contact_indices = CharacterizeContacts();
    if (contact_indices.empty() || contact_indices.size() >= 3) {
      return;
    }
    head_index_ = contact_indices.back();

    if (head_index_ < 23 || head_index_ > 27) {
      return;
    }

    head_strategy_count_down_ = 400;
  }
  if (head_strategy_count_down_ > 0) {

    for (int i = 0; i < robot_->motors.size(); ++i) {
      double desired_angle_dt = wave_params_.amplitudes[i] *
                                wave_params_.phases_dt[i] *
                                cos(wave_params_.phases[i]);
      double force =
          fabs(wave_params_.theta_dt[i]) < fabs(desired_angle_dt)
              ? 0.5 * desired_angle_dt *
                    exp(-(head_index_ - i) * (head_index_ - i) / 45.0)
              : 0;
      wave_params_.theta_dt[i] -= force;

      // wave_params_.phases_dt[i] -=
      //     3.5 * wave_params_.frequency *
      //     exp(-(head_index_ - i) * (head_index_ - i) / 30.0);
    }
  }
  if (head_strategy_count_down_ > -50) {
    head_strategy_count_down_--;
  }
}

void Controller::UpdatePhases(double dt) {
  const size_t kNumJoints = robot_->motors.size();
  for (size_t i = 0; i < kNumJoints; ++i) {
    if (head_strategy_count_down_ > 0) {
      // wave_params_.phases_dt[i] = wave_params_.frequency;
      // wave_params_.phases[i] += wave_params_.phases_dt[i] * dt;

    } else {
      wave_params_.phases_dt[i] = wave_params_.frequency;
      wave_params_.phases[i] += wave_params_.phases_dt[i] * dt;
      // wave_params_.desired_phases[i] += wave_params_.frequency * dt;
    }
  }

  // ApplyHeadStrategy();
  //
  // for (size_t i = 0; i < kNumJoints; ++i) {
  //   double error_phase =
  //       wave_params_.phases[i] - wave_params_.desired_phases[i];
  //   double error_phase_dt = wave_params_.phases_dt[i] -
  //   wave_params_.frequency;
  //   if (head_strategy_count_down_ > 0) {
  //
  //     wave_params_.phases[i] += dt * (wave_params_.phases_dt[i] -
  //                                     0.0 * error_phase - 0.0 *
  //                                     error_phase_dt);
  //   } else {
  //     // std::cout << i << " " << error_phase << "\n";
  //     wave_params_.phases[i] += dt * (wave_params_.phases_dt[i] -
  //                                     0.7 * error_phase - 0.7 *
  //                                     error_phase_dt);
  //   }
  // }
}

double DthLateralInhibition(const Eigen::VectorXd &torque, int index) {
  double K = 0.2;
  // Look at the nearby 5 joints
  double dth_dt = 0;
  double sigma = 4;
  for (int i = 0; i < torque.rows(); ++i) {
    dth_dt += torque(i) * exp(-(i - index) * (i - index) / sigma / sigma * 2);
  }
  dth_dt = std::min(std::max(dth_dt, -15.0), 15.0);
  return K * dth_dt;
}

void Controller::UpdateAngles(double dt) {
  const size_t kNumJoints = robot_->motors.size();
  for (size_t i = 0; i < kNumJoints; ++i) {
    // wave_params_.theta_dt[i] = wave_params_.amplitudes[i] *
    //                            wave_params_.phases_dt[i] *
    //                            cos(wave_params_.phases[i]);
    // wave_params_.theta[i] =
    //     wave_params_.amplitudes[i] * sin(wave_params_.phases[i]);
    double desired_angle =
        wave_params_.amplitudes[i] * sin(wave_params_.phases[i]);
    double desired_angle_dt = wave_params_.amplitudes[i] *
                              wave_params_.phases_dt[i] *
                              cos(wave_params_.phases[i]);
    double error_angle = wave_params_.theta[i] - desired_angle;
    double error_angle_dt = wave_params_.theta_dt[i] - desired_angle_dt;
    if (head_strategy_count_down_ > 0) {
      wave_params_.theta_dt[i] = 0.1 * error_angle - 0.1 * error_angle_dt;
    } else {
      wave_params_.theta_dt[i] =
          desired_angle_dt - 0.8 * error_angle - 0.8 * error_angle_dt;
    }
  }

  ApplyHeadStrategy();

  // std::cout << wave_params_.phases[26] << " " << wave_params_.theta[26]
  //           << std::endl;
  for (size_t i = 0; i < kNumJoints; ++i) {
    wave_params_.theta[i] += wave_params_.theta_dt[i] * dt;
  }
}

void Controller::ApplyWaveParams() {
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

void Controller::Step(double dt) {
  // ProcessCommandQueue(dt);
  // Will not overflow.
  steps_++;
  ExtractContactForces();
  // Update all window params according to the compliance parameter
  UpdateAmplitudes(dt);
  UpdatePhases(dt);
  UpdateAngles(dt);
  // Apply the window params.
  ApplyWaveParams();
  // Propagate the windows from head to tail
  PropagateWaveParams(dt);

  auto contact_positions = CharacterizeContacts();
}

size_t Controller::GetNumMotors() { return robot_->motors.size(); }

void Controller::EnablePIDMotorControl() {
  InitializeWaveParams();
  motor_functions_.resize(0);
  for (size_t i = 0; i < robot_->motors.size(); ++i) {
    motor_functions_.emplace_back(new ChFunctionMotor(
        wave_params_.amplitudes[i] * sin(wave_params_.phases[i]), 0));
    robot_->motors[i]->Initialize(motor_functions_[i],
                                  ChLinkEngine::ENG_MODE_TORQUE);
  }
}

void Controller::EnablePosMotorControl() {
  motor_functions_.resize(0);
  for (size_t i = 0; i < robot_->motors.size(); ++i) {
    motor_functions_.emplace_back(new ChFunctionMotor(
        wave_params_.amplitudes[i] * sin(wave_params_.phases[i]), 0));
    robot_->motors[i]->Initialize(motor_functions_[i],
                                  ChLinkEngine::ENG_MODE_ROTATION);
  }
}
