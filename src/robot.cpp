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

void ChronoRobotBuilder::BuildRobot() {
  ChSystem *ch_system = app_->GetSystem();
  {
    ChSharedBodyPtr ground(new ChBodyEasyBox(100, 100, 1, 1, false, false));
    ground->SetBodyFixed(true);
    ch_system->Add(ground);
    ChSharedBodyPtr foot(new ChBodyEasyBox(5, 5, 2, 1, true, true));
    ch_body_list_.push_back(foot.get());
    rft_body_list_.emplace_back(foot.get());
    rft_body_list_.back().Resize(100);
    ch_system->Add(foot);
    // Now Add linear actuation
    ChSharedPtr<ChLinkLinActuator> linear_actuator(new ChLinkLinActuator());
    linear_actuator->Initialize(foot, ground,
                                ChCoordsys<>(ChVector<>(0, 0, 0)));
    ch_system->Add(linear_actuator);
    controller_.AddEngine(linear_actuator.get());
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

void ChronoRobotBuilder::SetCollide(bool mcol) {
  for (int i = 0; i < ch_body_list_.size(); ++i) {
    ch_body_list_[i]->SetCollide(mcol);
  }
}
