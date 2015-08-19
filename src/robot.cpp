#include <array>

#include <unit_IRRLICHT/ChIrrApp.h>
#include <assets/ChColorAsset.h>
#include <physics/ChBodyEasy.h>
#include <physics/ChSystem.h>
#include "include/chfunction_squarewave.h"
#include "include/controller.h"
#include "include/robot.h"
#include "include/rft.h"

using namespace chrono;
using irr::ChIrrApp;

namespace {

// A Wedge is defined as a triangle (x-y plane) extruded in z direction
class Wedge {
  void SetPoint(size_t index, double x, double y);
  ChVector<> GetPoint(size_t index);
  ChVector<> BaryCenter();
  double depth;

private:
  std::array<ChVector<>, 3> points_;
};

void Wedge::SetPoint(size_t index, double x, double y) {
  index = index % 3;
  points_[index](0) = x;
  points_[index](1) = y;
}

ChVector<> Wedge::GetPoint(size_t index) {
  index = index % 3;
  return points_[index];
}

ChVector<> Wedge::BaryCenter() {
  return (points_[0] + points_[1] + points_[2]) / 3;
}

void MeshBox(ChVector<> sizes, std::vector<ChVector<> > &plist,
             std::vector<ChVector<> > &nlist, std::vector<double> &alist,
             std::vector<bool> &is_doubled_sided) {
  double lx = sizes(0);
  double ly = sizes(1);
  double lz = sizes(2);
  const size_t num_pieces = 30;
  plist.resize(num_pieces);
  nlist.resize(num_pieces);
  alist.resize(num_pieces);
  is_doubled_sided.resize(num_pieces);
  const int npiece = plist.size();
  for (int i = 0; i < npiece; ++i) {
    is_doubled_sided[i] = false;
    double denom = npiece / 2;
    if (i < denom) {
      double j = i + 0.5;
      plist[i].x = lx * j / denom - lx / 2.;
      plist[i].z = 0.5 * lz;
      nlist[i].z = 1.;
      alist[i] = lx * ly / denom;
    } else {
      double j = i + 0.5 - denom;
      plist[i].x = lx * j / denom - lx / 2.;
      plist[i].z = -0.5 * lz;
      nlist[i].z = -1.;
      alist[i] = lx * ly / denom;
    }
  }
}

void MeshWedge(ChVector<> p0, ChVector<> p1, ChVector<> p2, double depth,
               std::vector<ChVector<> > &plist, std::vector<ChVector<> > &nlist,
               std::vector<double> &alist,
               std::vector<bool> &is_doubled_sided) {
  const size_t num_pieces = 30;
  plist.reserve(num_pieces);
  nlist.reserve(num_pieces);
  alist.reserve(num_pieces);
  is_doubled_sided.reserve(num_pieces);
  const size_t np_per_side = num_pieces / 3;

  // first let us compute the
}

} // namespace

ChronoRobotBuilder::ChronoRobotBuilder(ChIrrApp *pApp) : app_(pApp) {}

double SegA(double s, double A) {
  // relative frame rotation between each seg;
  return A * cos(CH_C_2PI * s);
}

void ChronoRobotBuilder::BuildRobot(double depth, double alpha, double beta) {
  ChSystem *ch_system = app_->GetSystem();
  {
    ChSharedBodyPtr ground(new ChBodyEasyBox(100, 1, 100, 1, false, false));
    ground->SetBodyFixed(true);
    ch_system->Add(ground);

    // Create a wedge shape.
    std::vector<ChVector<> > points;
    double lx = 1.0;
    double ly = 0.3;
    double lz = 1.0;
    points.emplace_back(0.5 * lx, 0, lz * 0.5);
    points.emplace_back(-0.5 * lx, 0, lz * 0.5);
    points.emplace_back(0, ly, lz * 0.5);
    points.emplace_back(0.5 * lx, 0, -lz * 0.5);
    points.emplace_back(-0.5 * lx, 0, -lz * 0.5);
    points.emplace_back(0, ly, -lz * 0.5);
    ChSharedBodyPtr foot(new ChBodyEasyConvexHull(points, 1, true, true));
    foot->SetPos(ChVector<>(0, -depth, 0));
    foot->SetRot(Q_from_AngZ(alpha));
    rft_body_list_.emplace_back(foot.get());
    ch_system->Add(foot);

    // Now Add linear actuator.

    ChVector<> origin(0, 0, 0);
    ChQuaternion<> orientation = Q_from_AngX(CH_C_PI_2) * Q_from_AngY(beta);
    ChVector<> z_direction = orientation.Rotate(ChVector<>(0, 0, 1));

    ChSharedPtr<ChLinkLockPrismatic> prismatic(new ChLinkLockPrismatic());
    prismatic->Initialize(foot, ground, ChCoordsys<>(origin, orientation));
    ch_system->Add(prismatic);

    ChSharedPtr<ChLinkLinActuator> actuator(new ChLinkLinActuator());
    actuator->Initialize(
        foot, ground, false, ChCoordsys<>(foot->GetPos(), QUNIT),
        ChCoordsys<>(foot->GetPos() + 20.0 * z_direction, QUNIT));
    actuator->Set_lin_offset(20);
    ch_system->Add(actuator);
    controller_.AddEngine(actuator.get());
  }
}

ChVector<> ChronoRobotBuilder::GetRobotCoMPosition() {
  ChVector<> pos;
  const size_t nnode = rft_body_list_.size();
  double total_mass = 0;
  for (int i = 0; i < nnode; ++i) {
    ChVector<> tmppos = rft_body_list_[i].chbody->GetPos();
    double tmpmass = rft_body_list_[i].chbody->GetMass();
    pos += tmppos * tmpmass;
    total_mass += tmpmass;
  }
  return pos / total_mass;
}
