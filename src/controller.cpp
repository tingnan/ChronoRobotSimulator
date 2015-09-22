
#include "include/controller.h"
#include "include/robot.h"
#include "include/vector_utility.h"

using namespace chrono;

namespace {
ChVector<> ComputeAverageOrientation() { return ChVector<>(1.0, 0.0, 0.0); }
chrono::ChMatrixDynamic<> ComputeJacobian(Robot *robot) {
  const size_t kNumSegs = robot->body_list.size();
  chrono::ChVectorDynamic<> cum_theta(kNumSegs);
  for (size_t i = 0; i < kNumSegs; ++i) {
    auto body_ptr = robot->body_list[i];
    auto rot_quoternion = body_ptr->GetRot();
    double angle;
    ChVector<> axis;
    rot_quoternion.Q_to_AngAxis(angle, axis);
    if (axis(2) < 0) {
      angle = -angle;
    }
    cum_theta(i) = angle;
  }
  chrono::ChVectorDynamic<> theta = cum_theta;
  for (size_t i = 1; i < kNumSegs; ++i) {
    theta(i) = cum_theta(i) - cum_theta(i - 1);
  }

  chrono::ChVectorDynamic<> sin_thetas(kNumSegs);
  chrono::ChVectorDynamic<> cos_thetas(kNumSegs);
  for (size_t j = 0; j < kNumSegs; ++j) {
    // use cum_theta instead of theta for pendulumn
    sin_thetas(j) = sin(theta(j));
    cos_thetas(j) = cos(theta(j));
  }

  // Now compute the jacobian
  chrono::ChMatrixDynamic<> jacobian(3 * kNumSegs, kNumSegs);
  // compute the 3 row
  for (size_t i = 0; i < kNumSegs; ++i) {
    // Fill the column (partial x / partial theta_j)
    for (int j = i; j >= 0; --j) {
      double l = robot->body_length_list[j];
      if (j == i) {
        // the jth link is the end effector
        jacobian(3 * i + 0, j) = -0.5 * l * sin_thetas(j);
        jacobian(3 * i + 1, j) = 0.5 * l * cos_thetas(j);
      } else {
        jacobian(3 * i + 0, j) =
            -l * sin_thetas(j) + 0 * jacobian(3 * i + 0, j + 1);
        jacobian(3 * i + 1, j) =
            l * cos_thetas(j) + 0 * jacobian(3 * i + 1, j + 1);
      }
      jacobian(3 * i + 2, j) = 1;
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
    if (id_a >= 0 && id_a < contact_force_list_->size()) {
      (*contact_force_list_)[id_a] -= contact_force_normal;
    }

    auto id_b = model_b->GetPhysicsItem()->GetIdentifier();
    if (id_b >= 0 && id_b < contact_force_list_->size()) {
      (*contact_force_list_)[id_b] = contact_force_normal;
    }

    // std::cout << model_a->GetPhysicsItem()->GetIdentifier() << "  ";
    // std::cout << model_b->GetPhysicsItem()->GetIdentifier() << "\n";
    /*
    ChVector<> v1 = point_a;
    ChVector<> v2;

    switch (drawtype) {
    case ChIrrTools::CONTACT_DISTANCES:
      v2 = point_b;
      break;
    case ChIrrTools::CONTACT_NORMALS:
      v2 = point_a + vn * clen;
      break;
    case ChIrrTools::CONTACT_FORCES_N:
      v2 = point_a + vn * clen * react_forces.x;
      break;
    case ChIrrTools::CONTACT_FORCES:
      v2 = point_a + (mplanecoord * (react_forces * clen));
      break;
    }
    video::SColor mcol;
    if (distance > 0.0)
      mcol = video::SColor(200, 20, 255, 0); // green: non penetration
    else
      mcol = video::SColor(200, 255, 60, 60); // red: penetration

    this->cdriver->draw3DLine(core::vector3dfCH(v1), core::vector3dfCH(v2),
                              mcol);
    */
    return true; // to continue scanning contacts
  }

private:
  std::vector<ChVector<> > *contact_force_list_;
};

Controller::Controller(chrono::ChSystem *ch_system, class Robot *i_robot)
    : ch_system_(ch_system), robot_(i_robot),
      contact_force_list_(robot_->body_list.size()) {
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
  torques_ext_.Reset(kNumSegs);
  ChVector<> desired_direction = ComputeAverageOrientation();
  for (size_t i = 0; i < kNumSegs; ++i) {
    double force_mag = contact_force_list_[i].Length();
    double cos_theta = 0;
    if (force_mag > 1e-4) {
      cos_theta = desired_direction.Dot(contact_force_list_[i] / force_mag);
    }
    double weight = cos_theta;
    ext_force(3 *i + 0) = contact_force_list_[i](0) * weight;
    ext_force(3 *i + 1) = contact_force_list_[i](2) * weight;
    // Torque
    ext_force(3 *i + 2) = 0;
  }
  torques_ext_ = jacobian * ext_force;

  /*
  auto gravity = ch_system_->Get_G_acc().Length();
  ChVectorDynamic<> gravity_force(3 * kNumSegs);
  for (size_t i = 0; i < kNumSegs; ++i) {
    gravity_force(3 *i + 0) = 0;
    gravity_force(3 *i + 1) = robot_->body_list[i]->GetMass() * gravity;
    // Torque
    gravity_force(3 *i + 2) = 0;
  }
  */
  // torques_ext_ = jacobian * ext_force;
  /*
  for (size_t i = 0; i < torques_ext_.GetRows(); ++i) {
    std::cout << torques_ext_(i) << " ";
  }
  std::cout << "\n";
  */
}

size_t Controller::GetNumEngines() { return robot_->engine_list.size(); }
ChLinkEngine *Controller::GetEngine(size_t i) { return robot_->engine_list[i]; }

double Controller::GetExtTorque(size_t i, double t) { return torques_ext_(i); }

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
        double(i) * 2 / engine_list.size() * CH_C_2PI, 0.2, 0.5));
    engine_list[i]->Set_eng_mode(ChLinkEngine::ENG_MODE_ROTATION);
    engine_list[i]->Set_rot_funct(engine_funct);
    engine_list[i]->GetLimit_Rz()->Set_max(0.2);
    engine_list[i]->GetLimit_Rz()->Set_min(-0.2);
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
