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
    ChVector<> tmppos = rft_body_list_[i].GetChBody()->GetPos();
    double tmpmass = rft_body_list_[i].GetChBody()->GetMass();
    pos += tmppos * tmpmass;
    total_mass += tmpmass;
  }
  return pos / total_mass;
}
