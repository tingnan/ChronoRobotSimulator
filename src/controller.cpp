#include <Eigen/Dense>

#include "include/controller.h"
#include "include/robot.h"
#include "include/vector_utility.h"

using namespace chrono;

namespace {

double sigmoid(double sigma, double x) { return 1 / (1 + exp(-sigma * x)); }

double GetBodyXYPlaneAngle(ChBody *body_ptr) {
  double angle;
  ChVector<> axis;
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
ComputeChainJacobianTailFrame(const std::vector<ChBody *> &body_list,
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
ComputeChainJacobianCoMFrame(const std::vector<ChBody *> &body_list,
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
  // Fill the elements (partial x_i, y_i / partial theta_j)
  for (size_t i = 0; i < kNumSegs; ++i) {
    for (size_t j = 0; j < kNumSegs + 2; ++j) {
      jacobian(3 * i + 0, j) = jacobian_dx(i, j);
      jacobian(3 * i + 1, j) = jacobian_dy(i, j);
      jacobian(3 * i + 2, j) = jacobian_dw(i, j);
    }
  }
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
  std::cout << interal_torques.transpose() << std::endl;
  return interal_torques;
}

chrono::ChVectorDynamic<>
RedistContactWeight(const chrono::ChVectorDynamic<> &contact_weight) {
  // redistribute weight to
  // nearby joints
  const size_t kNumSegs = contact_weight.GetRows();
  chrono::ChVectorDynamic<> weight_redist(kNumSegs);

  /*
  double max_weight = 0;
  for (int i = 0; i < kNumSegs; ++i) {
    for (int j = 0; j < kNumSegs; ++j) {
      double decay = -(j - i) * (j - i) * 0.5;
      weight_redist(i) = weight_redist(i) + exp(decay) * contact_weight(j);
    }
    max_weight = std::max(max_weight, weight_redist(i));
  }

  if (max_weight > 0) {
    for (size_t i = 0; i < kNumSegs; ++i) {
      weight_redist(i) = weight_redist(i) / max_weight;
    }
  }
 */
  for (size_t i = 0; i < kNumSegs; ++i) {
    weight_redist(i) = 2 - contact_weight(i);
  }

  for (size_t i = 0; i < kNumSegs; ++i) {
    weight_redist(i) =
        weight_redist(i) > 1 ? 0.7 + 0.3 * weight_redist(i) : weight_redist(i);
  }

  return weight_redist;
}

} // namespoint_ace

class ExtractContactForce : public ChReportContactCallback2 {
public:
  ExtractContactForce(std::vector<ChVector<> > *contact_force_list)
      : contact_force_list_(contact_force_list) {}
  virtual bool ReportContactCallback2(const chrono::ChVector<> &point_a,
                                      const chrono::ChVector<> &point_b,
                                      const chrono::ChMatrix33<> &plane_coord,
                                      const double &distance,
                                      const chrono::ChVector<> &react_forces,
                                      const chrono::ChVector<> &react_torques,
                                      chrono::ChContactable *model_a,
                                      chrono::ChContactable *model_b) {
    ChVector<> contact_normal = plane_coord.Get_A_Xaxis();

    ChVector<> contact_force = plane_coord * react_forces;
    ChVector<> contact_force_normal = contact_normal * react_forces.x;
    auto contact_force_tangent =
        contact_force - dot(contact_normal, contact_force) * contact_normal;
    auto f = contact_force_tangent.Length();
    auto N = contact_force_normal.Length();
    auto id_a = model_a->GetPhysicsItem()->GetIdentifier();
    auto id_b = model_b->GetPhysicsItem()->GetIdentifier();

    if (id_a >= 0 && id_a < contact_force_list_->size() && id_b == -1) {
      (*contact_force_list_)[id_a] -= contact_force_normal;
    }

    if (id_b >= 0 && id_b < contact_force_list_->size() && id_a == -1) {
      (*contact_force_list_)[id_b] = contact_force_normal;
    }

    return true; // to continue scanning contacts
  }

private:
  std::vector<ChVector<> > *contact_force_list_;
};

Controller::Controller(chrono::ChSystem *ch_system, class Robot *i_robot)
    : ch_system_(ch_system), robot_(i_robot),
      contact_force_list_(robot_->body_list.size()),
      amplitudes_(robot_->engine_list.size()),
      average_contact_weight_(robot_->engine_list.size()) {
  contact_reporter_ = new ExtractContactForce(&contact_force_list_);
  for (size_t i = 0; i < amplitudes_.GetRows(); ++i) {
    amplitudes_(i) = default_amplitude_;
    average_contact_weight_(i) = 1;
  }
}

void Controller::Step(double dt) {

  // SolveChainInternalTorque();
  // exit(0);

  steps_++;
  const size_t kNumSegs = robot_->body_list.size();
  const size_t kNumJoints = robot_->engine_list.size();
  // the cycle is 1/f seconds and it takes 1/f/dt steps for a wave cycle. We
  // have kNumSegs - 1 joints and it takes about 1/f/dt/kNumJoints /
  // num_waves_ steps for the amplitudes to shift to next segments
  double cycle_time = CH_C_2PI / omega_;
  if (steps_ >= cycle_time / dt / kNumJoints / num_waves_) {
    for (size_t i = 0; i < kNumJoints; ++i) {
      average_contact_weight_(i) /= steps_;
      amplitudes_(i) = amplitudes_(i) * average_contact_weight_(i);

      average_contact_weight_(i) = 0;
    }
    // do a shift
    for (size_t i = 0; i < kNumJoints - 1; ++i) {
      amplitudes_(i) = amplitudes_(i + 1);
    }
    amplitudes_(kNumJoints - 1) = default_amplitude_;
    steps_ = 0;
  }

  // Clear all the forces in the contact force container
  for (auto &force : contact_force_list_) {
    force.Set(0);
  }
  ch_system_->GetContactContainer()->ReportAllContacts2(contact_reporter_);
  ChVectorDynamic<> forces_media(3 * kNumSegs);
  ChVectorDynamic<> forces_contact(3 * kNumSegs);
  ChVectorDynamic<> contact_weight(kNumSegs);
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

  // blend the contact weight and add the weight to amp
  contact_weight = RedistContactWeight(contact_weight);

  for (size_t i = 0; i < kNumJoints; ++i) {
    average_contact_weight_(i) +=
        0.5 * (contact_weight(i) + contact_weight(i + 1));
  }
  auto jacobian =
      ComputeChainJacobianCoMFrame(robot_->body_list, robot_->body_length_list);
  // torques_media_ = jacobian * forces_media;
  // torques_contact_ = jacobian * forces_contact;
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

// The parameters for the function
double ChFunctionController::Get_y(double t) {
  double torque = ComputeDriveTorque(t) + ComputeLimitTorque(t);
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
  // the torque at 0th index is for CoM, so we plus 1
  return -controller_->GetContactTorque(index_ + 1, t);
}
double ChFunctionController::GetMediaTorque(double t) {
  // the torque at 0th index is for CoM, so we plus 1
  return -controller_->GetMediaTorque(index_ + 1, t);
}

// The low level PID controller in motor.
double ChFunctionController::ComputeDriveTorque(double t) {
  t = 0;
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

void UsePositionControl(Robot *robot) {
  auto &engine_list = robot->engine_list;
  for (size_t i = 0; i < engine_list.size(); ++i) {
    ChSharedPtr<ChFunction_Sine> engine_funct(new ChFunction_Sine(
        double(i) * 2 / engine_list.size() * CH_C_2PI, 0.1, 0.6));
    engine_list[i]->Set_eng_mode(ChLinkEngine::ENG_MODE_ROTATION);
    engine_list[i]->Set_rot_funct(engine_funct);
  }
}

void UseController(Controller *controller) {
  for (size_t i = 0; i < controller->GetNumEngines(); ++i) {
    ChSharedPtr<ChFunctionController> engine_funct(
        new ChFunctionController(i, controller));
    controller->GetEngine(i)->Set_eng_mode(ChLinkEngine::ENG_MODE_TORQUE);
    controller->GetEngine(i)->Set_tor_funct(engine_funct);
  }
}
