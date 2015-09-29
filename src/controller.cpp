#include <Eigen/Dense>

#include "include/controller.h"
#include "include/robot.h"
#include "include/vector_utility.h"

using namespace chrono;

namespace {

double sigmoid(double sigma, double x) { return 1 / (1 + exp(-sigma * x)); }

chrono::ChMatrixDynamic<> ComputeJacobian(Robot *robot) {
  const size_t kNumSegs = robot->body_list.size();
  Eigen::VectorXd thetas(kNumSegs);
  thetas.setZero();
  for (size_t i = 0; i < kNumSegs; ++i) {
    auto body_ptr = robot->body_list[i];
    auto rot_quoternion = body_ptr->GetRot();
    double angle;
    ChVector<> axis;
    rot_quoternion.Q_to_AngAxis(angle, axis);
    if (axis(2) < 0) {
      angle = -angle;
    }
    thetas(i) = angle;
  }

  Eigen::VectorXd sin_thetas(kNumSegs);
  Eigen::VectorXd cos_thetas(kNumSegs);
  for (size_t j = 0; j < kNumSegs; ++j) {
    // use thetas instead of theta for pendulumn
    sin_thetas(j) = sin(thetas(j));
    cos_thetas(j) = cos(thetas(j));
  }

  // Now compute the jacobian, each line has n + 2 columns
  Eigen::MatrixXd jacobian_dx(kNumSegs, kNumSegs + 2);
  jacobian_dx.setZero();
  Eigen::MatrixXd jacobian_dy(kNumSegs, kNumSegs + 2);
  jacobian_dy.setZero();
  Eigen::MatrixXd jacobian_dt(kNumSegs, kNumSegs + 2);
  jacobian_dt.setZero();
  for (size_t i = 0; i < kNumSegs; ++i) {
    // the jth link is the end effector
    double l = robot->body_length_list[i];
    jacobian_dx(i, i) = 0 - 0.5 * l * sin_thetas(i);
    jacobian_dx(i, kNumSegs + 0) = 1;
    jacobian_dy(i, i) = 0 + 0.5 * l * cos_thetas(i);
    jacobian_dy(i, kNumSegs + 1) = 1;
    jacobian_dt(i, i) = 1;
    for (int j = i - 1; j >= 0; --j) {
      double l = robot->body_length_list[j];
      jacobian_dx(i, j) = jacobian_dx(i, j + 1) - l * sin_thetas(j);
      jacobian_dy(i, j) = jacobian_dy(i, j + 1) + l * cos_thetas(j);
      jacobian_dt(i, j) = 1;
    }
  }
  // std::cout << jacobian_dy << std::endl;
  // std::cout << jacobian_dt << std::endl;

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
  // std::cout << jj << std::endl;

  jacobian_dx = jacobian_dx * jj;
  jacobian_dy = jacobian_dy * jj;
  jacobian_dt = jacobian_dt * jj;

  // std::cout << jacobian_dy << std::endl;
  // std::cout << jacobian_dt << std::endl;

  chrono::ChMatrixDynamic<> jacobian(3 * kNumSegs, kNumSegs);
  // Fill the elements (partial x_i, y_i / partial theta_j)
  for (size_t i = 0; i < kNumSegs; ++i) {
    for (size_t j = 0; j < kNumSegs; ++j) {
      jacobian(3 * i + 0, j) = jacobian_dx(i, j);
      jacobian(3 * i + 1, j) = jacobian_dy(i, j);
      jacobian(3 * i + 2, j) = jacobian_dt(i, j);
    }
  }
  jacobian.MatrTranspose();
  return jacobian;
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
      weight_(robot_->body_list.size()) {
  contact_reporter_ = new ExtractContactForce(&contact_force_list_);
}

void Controller::Step(double dt) {
  auto jacobian = ComputeJacobian(robot_);
  const size_t kNumSegs = robot_->body_list.size();
  // Clear all the forces in the contact force container
  for (auto &force : contact_force_list_) {
    force.Set(0);
  }
  ch_system_->GetContactContainer()->ReportAllContacts2(contact_reporter_);

  ChVectorDynamic<> ext_force(3 * kNumSegs);
  ChVector<> desired_direction = ChVector<>(1.0, 0.0, 0.0);
  ChVector<> accum_force;
  for (size_t i = 0; i < kNumSegs; ++i) {
    // get the rft_force
    auto rft_force = robot_->body_list[i]->Get_accumulated_force();
    // std::cout << i << " " << rft_force << std::endl;
    accum_force += rft_force;
    // process contact forces;
    double force_mag = contact_force_list_[i].Length();
    double cos_theta = 1.0;
    if (force_mag > 1e-4) {
      contact_force_list_[i] = contact_force_list_[i] / force_mag;
      cos_theta = desired_direction.Dot(contact_force_list_[i]);
    }
    weight_(i) = (1 + cos_theta) * 0.5;
    // weight_(i) = sigmoid(6, -cos_theta);
    // fx
    ext_force(3 *i + 0) =
        contact_force_list_[i](0) * std::max(std::min(force_mag, 5.0), 0.0) +
        0 * rft_force(0);
    // fz
    ext_force(3 *i + 1) =
        contact_force_list_[i](2) * std::max(std::min(force_mag, 5.0), 0.0) +
        0 * rft_force(2);
    // Torque
    ext_force(3 *i + 2) = 0;
  }
  // std::cout << accum_force << std::endl;

  // redistribute weight to nearby joints
  ChVectorDynamic<> weight_redist(kNumSegs);
  double max_weight = 0;
  for (int i = 0; i < kNumSegs; ++i) {
    for (int j = 0; j < kNumSegs; ++j) {
      double decay = -(j - i) * (j - i) * 0.5;
      weight_redist(i) = weight_redist(i) + exp(decay) * weight_(j);
    }
    max_weight = std::max(max_weight, weight_redist(i));
  }

  if (max_weight > 0) {
    for (size_t i = 0; i < kNumSegs; ++i) {
      weight_(i) = weight_redist(i) / max_weight;
      // std::cout << weight_(i) << " ";
    }
    // std::cout << std::endl;
  }

  torques_ext_ = jacobian * ext_force;
}

size_t Controller::GetNumEngines() { return robot_->engine_list.size(); }
ChLinkEngine *Controller::GetEngine(size_t i) { return robot_->engine_list[i]; }

double Controller::GetExtTorque(size_t i, double t) { return torques_ext_(i); }
double Controller::GetExtTorqueWeight(size_t i, double t) { return weight_(i); }
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

void UsePositionControl(Robot *robot) {
  auto &engine_list = robot->engine_list;
  for (size_t i = 0; i < engine_list.size(); ++i) {
    ChSharedPtr<ChFunction_Sine> engine_funct(new ChFunction_Sine(
        double(i) * 2 / engine_list.size() * CH_C_2PI, 0.2, 0.6));
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
