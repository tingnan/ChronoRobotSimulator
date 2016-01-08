#include <array>
#include <random>
#include <algorithm>

#include <physics/ChSystem.h>
#include <physics/ChBodyEasy.h>
#include <chrono_irrlicht/ChIrrApp.h>

#include "json/json.h"
#include "include/vector_utility.h"
#include "include/robot.h"
#include "include/rft.h"
#include "include/controller.h"

using namespace chrono;

namespace {
const bool kEnableVisual = true;
const bool kEnableCollision = true;
const double kFriction = 0.3;
const double kDensity = 2700.0;

RFTMesh MeshRFTSquare(double lx, double ly, bool is_double_sided) {
  const size_t kMaxNumPieces = 100;
  size_t nx = 10, ny = 10;
  if (lx > ly) {
    nx = lx / ly * ny;
  } else {
    ny = ly / lx * nx;
  }
  double scale = double(nx * ny) / kMaxNumPieces;
  nx = std::max(nx * scale, 1.0);
  ny = std::max(ny * scale, 1.0);
  // Now create the mesh
  RFTMesh mesh;
  double area = (lx * ly) / (nx * ny);
  double wx = lx / nx;
  double wy = ly / ny;
  mesh.positions.reserve(nx * ny);
  mesh.normals.reserve(nx * ny);
  mesh.areas.reserve(nx * ny);
  mesh.is_double_sided.reserve(nx * ny);
  for (size_t i = 0; i < nx; ++i) {
    for (size_t j = 0; j < ny; ++j) {
      mesh.positions.emplace_back((i + 0.5) * wx - 0.5 * lx,
                                  (j + 0.5) * wy - 0.5 * ly, 0);
      mesh.normals.emplace_back(0, 0, 1);
      mesh.areas.push_back(area);
      mesh.is_double_sided.push_back(is_double_sided);
    }
  }

  return mesh;
}

void TransformRFTMesh(const ChFrame<> &frame, RFTMesh &mesh) {
  const size_t kNumPoints = mesh.positions.size();
  ChVector<> origin = frame.TransformLocalToParent(ChVector<>(0, 0, 0));
  for (size_t i = 0; i < kNumPoints; ++i) {
    mesh.positions[i] = frame.TransformLocalToParent(mesh.positions[i]);
    mesh.normals[i] = frame.TransformLocalToParent(mesh.normals[i]) - origin;
  }
}

} // namespace

void BuildWorld(chrono::ChSystem *ch_system, const Json::Value &params) {
  // Build a set of random collidables.
  if (true) {
    const size_t kGridSize = 50;
    const double kGridDist = 0.5;
    const double kHeight = 0.2;
    const double kSigma = 0.15;

    std::mt19937 generator(0);
    std::normal_distribution<double> normal_dist_radius(0.0, kSigma);

    for (size_t x_grid = 0; x_grid < kGridSize; ++x_grid) {
      for (size_t z_grid = 0; z_grid < kGridSize; ++z_grid) {
        double radius = fabs(normal_dist_radius(generator));
        // radius = 0.27;
        // radius = std::max(radius, 0.01);
        radius = 0.03;
        ChSharedPtr<ChBodyEasyCylinder> body_ptr(new ChBodyEasyCylinder(
            radius, kHeight, kDensity, kEnableCollision, kEnableVisual));
        body_ptr->SetBodyFixed(true);
        body_ptr->SetIdentifier(-1);
        body_ptr->GetMaterialSurface()->SetFriction(kFriction);
        double x_pos = x_grid * kGridDist;
        double z_pos = z_grid * kGridDist;
        body_ptr->SetPos(ChVector<>(x_pos, 0, z_pos));
        ch_system->Add(body_ptr);
      }
    }
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

Robot BuildRobot(chrono::ChSystem *ch_system, const Json::Value &params) {
  Robot i_robot;
  // Build a snake body.
  // if (false) {
  //   const size_t kNumSegments = 4;
  //   const double kW = 0.10;
  //   const double kLx = 1.0;
  //   ChVector<> center_pos(0, kLx * 0.5, 0);
  //   std::vector<ChSharedBodyPtr> body_container_;
  //   i_robot.inertia.resize(kNumSegments * 3, kNumSegments * 3);
  //   i_robot.inertia.setZero();
  //   for (size_t i = 0; i < kNumSegments; ++i) {
  //     i_robot.inertia(3 * i + 0, 3 *i + 0) = 1;
  //     i_robot.inertia(3 * i + 1, 3 *i + 1) = 1;
  //     i_robot.inertia(3 * i + 2, 3 *i + 2) = 1 / 12;
  //     ChSharedBodyPtr body_ptr;
  //     body_ptr = ChSharedBodyPtr(new ChBodyEasyBox(
  //         kLx, kW, kW, kDensity, kEnableCollision, kEnableVisual));
  //     body_ptr->SetPos(center_pos);
  //     body_ptr->SetIdentifier(i);
  //     body_ptr->GetMaterialSurface()->SetFriction(0.1);
  //     ch_system->Add(body_ptr);
  //     i_robot.body_list.push_back(body_ptr.get());
  //     i_robot.rft_body_list.emplace_back(body_ptr.get());
  //     // Buid the RFT body
  //     i_robot.rft_body_list.back().mesh = MeshRFTSquare(kLx, kW, true);
  //     i_robot.rft_body_list.back().forces.resize(
  //         i_robot.rft_body_list.back().mesh.positions.size());
  //     i_robot.body_length_list.push_back(kLx);
  //     body_container_.push_back(body_ptr);
  //     if (i > 0) {
  //       ChSharedPtr<ChLinkEngine> joint_ptr(new ChLinkEngine());
  //       ChVector<> position = center_pos - ChVector<>(0.5 * kLx, 0, 0);
  //       ChQuaternion<> orientation(Q_from_AngX(CH_C_PI_2));
  //       joint_ptr->Initialize(body_container_[i], body_container_[i - 1],
  //                             ChCoordsys<>(position, orientation));
  //       i_robot.engine_list.push_back(joint_ptr.get());
  //       joint_ptr->SetIdentifier(i);
  //       ch_system->Add(joint_ptr);
  //     }
  //     center_pos += ChVector<>(kLx, 0, 0);
  //   }
  // }

  if (true) {
    ChSharedPtr<ChBodyEasyBox> ground(new ChBodyEasyBox(
        500, 1.0, 500, 1.0, !kEnableCollision, !kEnableVisual));
    ground->SetBodyFixed(true);
    ground->SetPos(ChVector<>(0, -0.5, 0));
    ground->SetIdentifier(-2);
    ground->GetMaterialSurface()->SetFriction(0.0);
    ch_system->Add(ground);
    // the Snake params
    const size_t kNumSegments = 30;
    const double kL = 1.50;
    const double kW = 0.05;
    const double kLx = kL / kNumSegments;
    ChVector<> center_pos(0.10, -kW * 0.5, 2.60);
    std::vector<ChSharedBodyPtr> body_container_;
    i_robot.inertia.resize(kNumSegments * 3, kNumSegments * 3);
    i_robot.inertia.setZero();
    for (size_t i = 0; i < kNumSegments; ++i) {
      ChSharedBodyPtr body_ptr;
      if (i == kNumSegments - 1) {
        body_ptr = ChSharedBodyPtr(new ChBodyEasyCylinder(
            kW * 0.5, kW, kDensity, kEnableCollision, kEnableVisual));
        i_robot.body_length_list.push_back(kW);
      } else {
        body_ptr = ChSharedBodyPtr(new ChBodyEasyBox(
            kLx, kW, kW, kDensity, kEnableCollision, kEnableVisual));
        i_robot.body_length_list.push_back(kLx);
      }
      i_robot.inertia(3 * i + 0, 3 *i + 0) = body_ptr->GetMass();
      i_robot.inertia(3 * i + 1, 3 *i + 1) = body_ptr->GetMass();
      i_robot.inertia(3 * i + 2, 3 *i + 2) = body_ptr->GetInertiaXX().z;
      body_ptr->SetPos(center_pos);
      body_ptr->SetIdentifier(i);
      body_ptr->GetMaterialSurface()->SetFriction(kFriction);
      ch_system->Add(body_ptr);
      i_robot.body_list.push_back(body_ptr.get());
      i_robot.rft_body_list.emplace_back(body_ptr.get());
      // Buid the RFT body
      i_robot.rft_body_list.back().mesh = MeshRFTSquare(kLx, kW, true);
      i_robot.rft_body_list.back().forces.resize(
          i_robot.rft_body_list.back().mesh.positions.size());
      body_container_.push_back(body_ptr);
      // The engines.
      if (i > 0) {
        ChSharedPtr<ChLinkEngine> joint_ptr(new ChLinkEngine());
        ChVector<> position = center_pos - ChVector<>(0.5 * kLx, 0, 0);
        ChQuaternion<> orientation(Q_from_AngX(CH_C_PI_2));
        joint_ptr->Initialize(body_container_[i], body_container_[i - 1],
                              ChCoordsys<>(position, orientation));
        i_robot.engine_list.push_back(joint_ptr.get());
        joint_ptr->SetIdentifier(i);
        ch_system->Add(joint_ptr);
      }

      if (true) {
        // The joints that maintain the snake in plane.
        ChSharedPtr<ChLinkLockPlanePlane> inplanelink(new ChLinkLockPlanePlane);
        inplanelink->Initialize(
            ground, body_ptr,
            ChCoordsys<>(ChVector<>(), Q_from_AngX(CH_C_PI_2)));
        ch_system->Add(inplanelink);
      }

      if (false) {
        // Add a cylinder at the joint.
        ChSharedPtr<ChBodyEasyCylinder> cylinder_ptr(new ChBodyEasyCylinder(
            kW * 0.5, kW, kDensity, kEnableCollision, kEnableVisual));
        cylinder_ptr->SetPos(center_pos + ChVector<>(kLx * 0.5, 0, 0));
        cylinder_ptr->SetIdentifier(i + 100);
        ch_system->Add(cylinder_ptr);
        ChSharedPtr<ChLinkLockLock> link(new ChLinkLockLock);
        link->Initialize(body_ptr, cylinder_ptr, ChCoordsys<>(VNULL));
        ch_system->Add(link);
      }

      center_pos += ChVector<>(kLx, 0, 0);
    }
  }

  return i_robot;
}
