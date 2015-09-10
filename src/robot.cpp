#include <array>
#include <random>

#include <physics/ChSystem.h>
#include <physics/ChBodyEasy.h>
#include <unit_IRRLICHT/ChIrrApp.h>

#include "json/json.h"
#include "include/robot.h"
#include "include/rft.h"
#include "include/chfunction_controller.h"

using namespace chrono;

Robot BuildRobotAndWorld(irr::ChIrrApp *ch_app, const Json::Value &params) {
  auto ch_system = ch_app->GetSystem();
  Robot i_robot;
  const double kDensity = 1.0;
  const bool kEnableCollision = true;
  const bool kEnableVisual = true;

  // Build a snake body.
  {
    ChSharedPtr<ChBodyEasyBox> ground(
        new ChBodyEasyBox(500, 0.1, 500, 1.0, false, false));
    ground->SetBodyFixed(true);
    ch_system->Add(ground);
    // the Snake params
    const size_t kNumSegments = 25;
    const double kL = 1.10;
    const double kW = 0.05;
    const double kLx = kL / kNumSegments;
    ChVector<> center_pos(-kL - 0.1, 0, 0);
    std::vector<ChSharedBodyPtr> body_container_;
    for (size_t i = 0; i < kNumSegments; ++i) {
      ChSharedPtr<ChBodyEasyBox> body_ptr(new ChBodyEasyBox(
          kLx, kW, kW, kDensity, kEnableCollision, kEnableVisual));
      body_ptr->SetPos(center_pos);
      body_ptr->SetIdentifier(i);
      ch_system->Add(body_ptr);
      i_robot.body_list.push_back(body_ptr.get());
      body_container_.push_back(body_ptr);
      // The engines.
      if (i > 0) {
        ChSharedPtr<ChLinkEngine> joint_ptr(new ChLinkEngine());
        ChVector<> position = center_pos - ChVector<>(0.5 * kLx, 0, 0);
        ChQuaternion<> orientation(Q_from_AngX(CH_C_PI_2));
        joint_ptr->Initialize(body_container_[i], body_container_[i - 1],
                              ChCoordsys<>(position, orientation));
        i_robot.engine_list.push_back(joint_ptr.get());
        joint_ptr->SetIdentifier(i - 1);
        ch_system->Add(joint_ptr);
      }
      // The joints that maintain the snake in plane
      ChSharedPtr<ChLinkLockPlanePlane> inplanelink(new ChLinkLockPlanePlane);
      inplanelink->Initialize(
          ground, body_container_[i],
          ChCoordsys<>(ChVector<>(), Q_from_AngX(CH_C_PI_2)));
      ch_system->Add(inplanelink);

      center_pos += ChVector<>(kLx, 0, 0);
    }
  }

  // Build a set of random collidables.
  {
    const size_t kGridSize = 20;
    const double kGridDist = 0.5;
    const double kHeight = 0.2;
    const double kSigma = 0.1;

    std::mt19937 generator(time(0));
    std::normal_distribution<double> normal_dist_radius(0.0, kSigma);

    for (size_t x_grid = 0; x_grid < kGridSize; ++x_grid) {
      for (size_t z_grid = 0; z_grid < kGridSize; ++z_grid) {
        double radius = fabs(normal_dist_radius(generator));
        ChSharedPtr<ChBodyEasyCylinder> body_ptr(new ChBodyEasyCylinder(
            radius, kHeight, kDensity, kEnableCollision, kEnableVisual));
        body_ptr->SetBodyFixed(true);
        body_ptr->SetIdentifier(-1);
        double x_pos = x_grid * kGridDist;
        double z_pos = (z_grid - 0.5 * kGridSize) * kGridDist;
        body_ptr->SetPos(ChVector<>(x_pos, 0, z_pos));
        ch_system->Add(body_ptr);
      }
    }
  }

  // Do visual binding.
  ch_app->AssetBindAll();
  ch_app->AssetUpdateAll();

  return i_robot;
}
