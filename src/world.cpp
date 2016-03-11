#include <physics/ChBodyEasy.h>
#include <physics/ChSystem.h>

#include <random>

#include <physics/ChBodyEasy.h>
#include <physics/ChSystem.h>

#include "include/world.h"
#include "json/json.h"

using namespace chrono;

void BuildWorld(ChSystem *ch_system, const Json::Value &params) {
  // Build a set of random collidables.
  if (true) {
    const bool kEnableVisual = true;
    const bool kEnableCollision = true;
    const double kFriction = 0.0;
    const double kDensity = 5000.0;

    const double kGridDist = params["spacing"].asDouble();
    const size_t kGridSize = kGridDist < 0.35 ? 50 : 30;
    const double kHeight = 0.2;
    const double kSigma = 0.08;
    const double kRadius = 0.03;
    std::shared_ptr<ChBodyEasyCylinder> body_ptr(new ChBodyEasyCylinder(
        kRadius, kHeight, kDensity, kEnableCollision, kEnableVisual));
    body_ptr->SetBodyFixed(true);
    body_ptr->SetIdentifier(-1);
    body_ptr->GetMaterialSurface()->SetFriction(kFriction);
    body_ptr->SetPos(ChVector<>(0, 0, 0));
    ch_system->Add(body_ptr);

    // std::mt19937 generator(1);
    // std::normal_distribution<double> normal_dist_radius(0.0, kSigma);

    // for (size_t x_grid = 0; x_grid < kGridSize; ++x_grid) {
    //   for (size_t z_grid = 0; z_grid < kGridSize; ++z_grid) {
    //     double radius = fabs(normal_dist_radius(generator));
    //     // radius = 0.27;
    //     // radius = std::max(radius, 0.01);
    //     radius = 0.03;
    //     std::shared_ptr<ChBodyEasyCylinder> body_ptr(new ChBodyEasyCylinder(
    //         radius, kHeight, kDensity, kEnableCollision, kEnableVisual));
    //     body_ptr->SetBodyFixed(true);
    //     body_ptr->SetIdentifier(-1);
    //     body_ptr->GetMaterialSurface()->SetFriction(kFriction);
    //     double x_pos = x_grid * kGridDist + 0.0 *
    //     normal_dist_radius(generator);
    //     double z_pos = z_grid * kGridDist - 0.5 * kGridSize * kGridDist +
    //                    0.0 * normal_dist_radius(generator);
    //     body_ptr->SetPos(ChVector<>(x_pos, 0, z_pos));
    //     ch_system->Add(body_ptr);
    //   }
    // }
  }

  // if (false) {
  //   const double kAngle = CH_C_PI_2;
  //   const double kTunnelW = 0.1;
  //   const double kTunnelH = 0.1;
  //   const double kTunnelL = 1.5;
  //   {
  //     std::vector<ChVector<> > points;
  //     points.emplace_back(0, -kTunnelH, kTunnelW / 2);
  //     points.emplace_back(0, -kTunnelH, kTunnelW / 2 + kTunnelL);
  //     points.emplace_back(kTunnelL * (1 + cos(kAngle)), -kTunnelH,
  //                         kTunnelW / 2 + kTunnelL * sin(kAngle));
  //     points.emplace_back(kTunnelL, -kTunnelH, kTunnelW / 2);
  //
  //     points.emplace_back(0, kTunnelH, kTunnelW / 2);
  //     points.emplace_back(0, kTunnelH, kTunnelW / 2 + kTunnelL);
  //     points.emplace_back(kTunnelL * (1 + cos(kAngle)), kTunnelH,
  //                         kTunnelW / 2 + kTunnelL * sin(kAngle));
  //     points.emplace_back(kTunnelL, kTunnelH, kTunnelW / 2);
  //
  //     ChSharedBodyPtr wall(
  //         new ChBodyEasyConvexHullAuxRef(points, 1.0, true, true));
  //     wall->SetBodyFixed(true);
  //     wall->GetMaterialSurface()->SetFriction(kFriction);
  //     ch_system->Add(wall);
  //   }
  //
  //   {
  //     std::vector<ChVector<> > points;
  //     points.emplace_back(0, -kTunnelH, -kTunnelW / 2);
  //     points.emplace_back(0, -kTunnelH, -3 * kTunnelW / 2);
  //     points.emplace_back(kTunnelL + 2 * kTunnelW * tan(kAngle / 2),
  // -kTunnelH,
  //                         -3 * kTunnelW / 2);
  //     points.emplace_back(kTunnelL + kTunnelW * tan(kAngle / 2), -kTunnelH,
  //                         -kTunnelW / 2);
  //
  //     points.emplace_back(0, kTunnelH, -kTunnelW / 2);
  //     points.emplace_back(0, kTunnelH, -3 * kTunnelW / 2);
  //     points.emplace_back(kTunnelL + 2 * kTunnelW * tan(kAngle / 2),
  // kTunnelH,
  //                         -3 * kTunnelW / 2);
  //     points.emplace_back(kTunnelL + kTunnelW * tan(kAngle / 2), kTunnelH,
  //                         -kTunnelW / 2);
  //     ChSharedBodyPtr wall(
  //         new ChBodyEasyConvexHullAuxRef(points, 1.0, true, true));
  //     wall->SetBodyFixed(true);
  //     wall->GetMaterialSurface()->SetFriction(kFriction);
  //     ch_system->Add(wall);
  //   }
  //
  //   {
  //     std::vector<ChVector<> > points;
  //     points.emplace_back(kTunnelL + kTunnelW * tan(kAngle / 2), -kTunnelH,
  //                         -kTunnelW / 2);
  //     points.emplace_back(kTunnelL + 2 * kTunnelW * tan(kAngle / 2),
  // -kTunnelH,
  //                         -3 * kTunnelW / 2);
  //     points.emplace_back(
  //         (kTunnelL + 2 * kTunnelW * tan(kAngle / 2)) * (1 + cos(kAngle)),
  //         -kTunnelH,
  //         -3 * kTunnelW / 2 +
  //             (kTunnelL + 2 * kTunnelW * tan(kAngle / 2)) * sin(kAngle));
  //     points.emplace_back(
  //         (kTunnelL + kTunnelW * tan(kAngle / 2)) * (1 + cos(kAngle)),
  //         -kTunnelH, -kTunnelW / 2 +
  //                        (kTunnelL + kTunnelW * tan(kAngle / 2)) *
  // sin(kAngle));
  //
  //     points.emplace_back(kTunnelL + kTunnelW * tan(kAngle / 2), kTunnelH,
  //                         -kTunnelW / 2);
  //     points.emplace_back(kTunnelL + 2 * kTunnelW * tan(kAngle / 2),
  // kTunnelH,
  //                         -3 * kTunnelW / 2);
  //     points.emplace_back(
  //         (kTunnelL + 2 * kTunnelW * tan(kAngle / 2)) * (1 + cos(kAngle)),
  //         kTunnelH,
  //         -3 * kTunnelW / 2 +
  //             (kTunnelL + 2 * kTunnelW * tan(kAngle / 2)) * sin(kAngle));
  //     points.emplace_back(
  //         (kTunnelL + kTunnelW * tan(kAngle / 2)) * (1 + cos(kAngle)),
  // kTunnelH,
  //         -kTunnelW / 2 +
  //             (kTunnelL + kTunnelW * tan(kAngle / 2)) * sin(kAngle));
  //
  //     ChSharedBodyPtr wall(
  //         new ChBodyEasyConvexHullAuxRef(points, 1.0, true, true));
  //     wall->SetBodyFixed(true);
  //     wall->GetMaterialSurface()->SetFriction(kFriction);
  //     ch_system->Add(wall);
  //   }
  // }
}
