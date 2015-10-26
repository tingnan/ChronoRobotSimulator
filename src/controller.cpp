#include <Eigen/Dense>

#include "include/controller.h"
#include "include/chfunction_controller.h"
#include "include/robot.h"
#include "include/vector_utility.h"
#include "include/contact_reporter.h"

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

Eigen::MatrixXd ChainJacobianDx(const std::vector<double> &body_length_list,
                                const Eigen::VectorXd &sin_thetas) {
  const size_t kNumSegs = body_length_list.size();
  Eigen::MatrixXd jacobian_dx(kNumSegs, kNumSegs + 2);
  jacobian_dx.setZero();
  for (size_t i = 0; i < kNumSegs; ++i) {
    double l = body_length_list[i];
    jacobian_dx(i, i) = -0.5 * l * sin_thetas(i);
    jacobian_dx(i, kNumSegs) = 1;
    for (int j = i - 1; j >= 0; --j) {
      double l = body_length_list[j];
      jacobian_dx(i, j) = jacobian_dx(i, j + 1) - l * sin_thetas(j);
    }
  }
  return jacobian_dx;
}

Eigen::MatrixXd ChainJacobianDy(const std::vector<double> &body_length_list,
                                const Eigen::VectorXd &cos_thetas) {
  const size_t kNumSegs = body_length_list.size();
  Eigen::MatrixXd jacobian_dy(kNumSegs, kNumSegs + 2);
  jacobian_dy.setZero();
  for (size_t i = 0; i < kNumSegs; ++i) {
    double l = body_length_list[i];
    jacobian_dy(i, i) = 0.5 * l * cos_thetas(i);
    jacobian_dy(i, kNumSegs + 1) = 1;
    for (int j = i - 1; j >= 0; --j) {
      double l = body_length_list[j];
      jacobian_dy(i, j) = jacobian_dy(i, j + 1) + l * cos_thetas(j);
    }
  }
  return jacobian_dy;
}

Eigen::MatrixXd ChainJacobianDw(const std::vector<double> &body_length_list) {
  const size_t kNumSegs = body_length_list.size();
  Eigen::MatrixXd jacobian_dw(kNumSegs, kNumSegs + 2);
  jacobian_dw.setZero();
  for (size_t i = 0; i < kNumSegs; ++i) {
    for (size_t j = 0; j <= i; ++j) {
      jacobian_dw(i, j) = 1;
    }
  }
  return jacobian_dw;
}

Eigen::MatrixXd
ComputeChainJacobianTailFrame(const std::vector<chrono::ChBody *> &body_list,
                              const std::vector<double> &body_length_list) {
  const size_t kNumSegs = body_list.size();
  Eigen::VectorXd thetas(kNumSegs);
  thetas.setZero();
  for (size_t i = 0; i < kNumSegs; ++i) {
    thetas(i) = GetBodyXYPlaneAngle(body_list[i]);
  }

  Eigen::VectorXd sin_thetas(kNumSegs);
  Eigen::VectorXd cos_thetas(kNumSegs);
  for (size_t j = 0; j < kNumSegs; ++j) {
    sin_thetas(j) = sin(thetas(j));
    cos_thetas(j) = cos(thetas(j));
  }
  auto jacobian_dx = ChainJacobianDx(body_length_list, sin_thetas);
  auto jacobian_dy = ChainJacobianDy(body_length_list, cos_thetas);
  auto jacobian_dw = ChainJacobianDw(body_length_list);

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

Eigen::MatrixXd
ComputeChainJacobianCoMFrame(const std::vector<chrono::ChBody *> &body_list,
                             const std::vector<double> &body_length_list) {
  const size_t kNumSegs = body_list.size();
  Eigen::VectorXd thetas(kNumSegs);
  thetas.setZero();
  for (size_t i = 0; i < kNumSegs; ++i) {
    thetas(i) = GetBodyXYPlaneAngle(body_list[i]);
  }

  Eigen::VectorXd sin_thetas(kNumSegs);
  Eigen::VectorXd cos_thetas(kNumSegs);
  for (size_t j = 0; j < kNumSegs; ++j) {
    sin_thetas(j) = sin(thetas(j));
    cos_thetas(j) = cos(thetas(j));
  }
  auto jacobian_dx = ChainJacobianDx(body_length_list, sin_thetas);
  auto jacobian_dy = ChainJacobianDy(body_length_list, cos_thetas);
  auto jacobian_dw = ChainJacobianDw(body_length_list);

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
} // namespace

using namespace chrono;

Controller::Controller(chrono::ChSystem *ch_system, class Robot *i_robot)
    : ch_system_(ch_system), robot_(i_robot),
      contact_force_list_(robot_->body_list.size()),
      amplitudes_(robot_->engine_list.size()),
      average_contact_weight_(robot_->engine_list.size()) {
  contact_reporter_ = new ExtractContactForce(&contact_force_list_);
  for (size_t i = 0; i < amplitudes_.rows(); ++i) {
    amplitudes_(i) = default_amplitude_;
    average_contact_weight_(i) = 1;
  }
}

void Controller::Step(double dt) {

  // auto jacobian = ComputeChainJacobianTailFrame(robot_->body_list,
  //                                               robot_->body_length_list);
  auto jacobian =
      ComputeChainJacobianCoMFrame(robot_->body_list, robot_->body_length_list);
  // steps_++;
  const size_t kNumSegs = robot_->body_list.size();
  const size_t kNumJoints = robot_->engine_list.size();

  // Clear all the forces in the contact force
  // container
  for (auto &force : contact_force_list_) {
    force.Set(0);
  }
  ch_system_->GetContactContainer()->ReportAllContacts2(contact_reporter_);

  Eigen::VectorXd forces_media(3 * kNumSegs);
  Eigen::VectorXd forces_contact(3 * kNumSegs);
  Eigen::VectorXd contact_weight(kNumSegs);
  ChVector<> desired_direction = ChVector<>(1.0, 0.0, 0.0);
  ChVector<> accum_force;
  for (size_t i = 0; i < kNumSegs; ++i) {
    // get the rft_force
    auto rft_force = robot_->body_list[i]->Get_accumulated_force();
    accum_force += rft_force;

    // process contact forces;
    double cos_theta = 0.0;
    double force_mag = contact_force_list_[i].Length();
    if (force_mag > 1e-4) {
      contact_force_list_[i] = contact_force_list_[i] / force_mag;
      cos_theta = desired_direction.Dot(contact_force_list_[i]);
    }

    contact_weight(i) = (2 - 2 * sigmoid(5, cos_theta));
    // fx
    forces_contact(3 *i + 0) =
        contact_force_list_[i](0) * std::max(std::min(force_mag, 5.0), 0.0);
    forces_media(3 *i + 0) = rft_force(0);
    // fz
    forces_contact(3 *i + 1) =
        contact_force_list_[i](2) * std::max(std::min(force_mag, 5.0), 0.0);
    forces_media(3 *i + 1) = rft_force(2);
    // Torque
    forces_contact(3 *i + 2) = 0;
    forces_media(3 *i + 2) = 0;
  }
  auto torque_int =
      SolveChainInternalTorque(robot_->inertia, jacobian, forces_media);
  torques_media_ = torque_int.block(1, 0, kNumJoints, 1);
}

size_t Controller::GetNumEngines() { return robot_->engine_list.size(); }
ChLinkEngine *Controller::GetEngine(size_t i) { return robot_->engine_list[i]; }

double Controller::GetMediaTorque(size_t index, double t) {
  return torques_media_(index);
}

double Controller::GetContactTorque(size_t index, double t) {
  return torques_contact_(index);
}

double Controller::GetAngle(size_t index, double t) {
  const double phase =
      double(index * num_waves_) / robot_->engine_list.size() * CH_C_2PI;
  double desired_angle = default_amplitude_ * sin(omega_ * t + phase);
  return desired_angle;
}

double Controller::GetAngularSpeed(size_t index, double t) {
  const double phase =
      double(index * num_waves_) / robot_->engine_list.size() * CH_C_2PI;
  double desired_angular_speed =
      default_amplitude_ * omega_ * cos(omega_ * t + phase);
  return desired_angular_speed;
}

void Controller::UsePositionControl() {
  auto &engine_list = robot_->engine_list;
  for (size_t i = 0; i < engine_list.size(); ++i) {
    ChSharedPtr<ChFunction_Sine> engine_funct(new ChFunction_Sine(
        double(i * num_waves_) / engine_list.size() * CH_C_2PI,
        omega_ / CH_C_2PI, default_amplitude_));
    engine_list[i]->Set_eng_mode(ChLinkEngine::ENG_MODE_ROTATION);
    engine_list[i]->Set_rot_funct(engine_funct);
  }
}

void Controller::UseForceControl() {
  for (size_t i = 0; i < GetNumEngines(); ++i) {
    ChSharedPtr<ChFunctionController> engine_funct(
        new ChFunctionController(i, this));
    GetEngine(i)->Set_eng_mode(ChLinkEngine::ENG_MODE_TORQUE);
    GetEngine(i)->Set_tor_funct(engine_funct);
  }
}
