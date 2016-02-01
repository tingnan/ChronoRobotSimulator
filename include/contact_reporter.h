#ifndef INCLUDE_CONTACT_REPORTER_H_
#define INCLUDE_CONTACT_REPORTER_H_

#include <physics/ChContactContainerBase.h>

#include "include/vector_utility.h"

namespace chrono {

class ContactSerializer : public ChReportContactCallback2 {
public:
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
  }
};

class ContactExtractor : public ChReportContactCallback2 {
public:
  explicit ContactExtractor(size_t num_bodies) : contact_forces_(num_bodies) {}
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

    if (id_a >= 0 && id_a < contact_forces_.size()) {
      contact_forces_[id_a] -= contact_force_normal;
    }

    if (id_b >= 0 && id_b < contact_forces_.size()) {
      contact_forces_[id_b] = contact_force_normal;
    }

    return true; // to continue scanning contacts
  }

  void Reset() {
    for (auto &force : contact_forces_) {
      force.SetNull();
    }
  }
  std::vector<ChVector<>> GetContactForces() { return contact_forces_; }

private:
  std::vector<ChVector<>> contact_forces_;
};
} // namespace chrono

#endif // INCLUDE_CONTACT_REPORTER_H_
