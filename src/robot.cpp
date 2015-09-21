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

Robot BuildRobotAndWorld(irr::ChIrrApp *ch_app, const Json::Value &params) {
  auto ch_system = ch_app->GetSystem();
  Robot i_robot;
  const double kDensity = 2700.0;
  const bool kEnableCollision = true;
  const bool kEnableVisual = true;

  if (true) {
    ChSharedPtr<ChBodyEasyBox> ground(new ChBodyEasyBox(
        500, 1.0, 500, 1.0, !kEnableCollision, !kEnableVisual));
    ground->SetBodyFixed(true);
    ground->SetPos(ChVector<>(0, -0.6, 0));
    ground->SetIdentifier(-1);
    ground->GetMaterialSurface()->SetFriction(0.1);
    ch_system->Add(ground);

    const double kL = 0.20;
    const double kW = 0.02;
    ChSharedPtr<ChBodyEasyBox> link_1(new ChBodyEasyBox(
        kL, kW, kW, kDensity, kEnableCollision, kEnableVisual));
    link_1->SetPos(ChVector<>(kL * 0.5, 0, 0));
    ch_system->Add(link_1);
    i_robot.body_list.push_back(link_1.get());
    i_robot.body_length_list.push_back(kL);

    ChSharedPtr<ChLinkEngine> joint_1(new ChLinkEngine);
    joint_1->Initialize(link_1, ground,
                        ChCoordsys<>(ChVector<>(), Q_from_AngX(CH_C_PI_2)));
    ch_system->Add(joint_1);
    i_robot.engine_list.push_back(joint_1.get());

    ChSharedPtr<ChBodyEasyBox> link_2(new ChBodyEasyBox(
        kL, kW, kW, kDensity, kEnableCollision, kEnableVisual));
    link_2->SetPos(ChVector<>(kL * 1.5, 0, 0));
    ch_system->Add(link_2);
    i_robot.body_list.push_back(link_2.get());
    i_robot.body_length_list.push_back(kL);

    ChSharedPtr<ChLinkEngine> joint_2(new ChLinkEngine);
    joint_2->Initialize(link_2, link_1, ChCoordsys<>(ChVector<>(kL, 0, 0),
                                                     Q_from_AngX(CH_C_PI_2)));
    ch_system->Add(joint_2);
    i_robot.engine_list.push_back(joint_2.get());

    if (true) {
      ChSharedPtr<ChBodyEasyBox> link_1(
          new ChBodyEasyBox(kW, kW, kW, 1.0, !kEnableCollision, kEnableVisual));
      link_1->SetPos(ChVector<>(4 * kL, 0, 0));
      link_1->SetBodyFixed(true);
      link_1->AddAsset(ChSharedPtr<ChColorAsset>(new ChColorAsset(1, 0, 0)));
      ch_system->Add(link_1);

      ChSharedPtr<ChBodyEasyBox> link_2(
          new ChBodyEasyBox(kW, kW, kW, 1.0, !kEnableCollision, kEnableVisual));
      link_2->SetPos(ChVector<>(0, 4 * kL, 0));
      link_2->SetBodyFixed(true);
      link_2->AddAsset(ChSharedPtr<ChColorAsset>(new ChColorAsset(0, 1, 0)));
      ch_system->Add(link_2);

      ChSharedPtr<ChBodyEasyBox> link_3(
          new ChBodyEasyBox(kW, kW, kW, 1.0, !kEnableCollision, kEnableVisual));
      link_3->SetPos(ChVector<>(0, 0, 4 * kL));
      link_3->SetBodyFixed(true);
      link_3->AddAsset(ChSharedPtr<ChColorAsset>(new ChColorAsset(0, 0, 1)));
      ch_system->Add(link_3);
    }
  }

  // Build a snake body.
  if (false) {
    ChSharedPtr<ChBodyEasyBox> ground(new ChBodyEasyBox(
        500, 1.0, 500, 1.0, !kEnableCollision, !kEnableVisual));
    ground->SetBodyFixed(true);
    ground->SetPos(ChVector<>(0, -0.6, 0));
    ground->SetIdentifier(-1);
    ground->GetMaterialSurface()->SetFriction(0.1);
    ch_system->Add(ground);
    // the Snake params
    const size_t kNumSegments = 25;
    const double kL = 1.10;
    const double kW = 0.05;
    const double kLx = kL / kNumSegments;
    ChVector<> center_pos(-kL * 0.8, -kW, 0);
    std::vector<ChSharedBodyPtr> body_container_;
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
      body_ptr->SetPos(center_pos);
      body_ptr->SetIdentifier(i);
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
        joint_ptr->SetIdentifier(i - 1);
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

  // Build a set of random collidables.
  if (false) {
    const size_t kGridSize = 5;
    const double kGridDist = 0.3;
    const double kHeight = 0.2;
    const double kSigma = 0.1;

    std::mt19937 generator(time(0));
    std::normal_distribution<double> normal_dist_radius(0.0, kSigma);

    for (size_t x_grid = 0; x_grid < kGridSize; ++x_grid) {
      for (size_t z_grid = 0; z_grid < kGridSize; ++z_grid) {
        double radius = fabs(normal_dist_radius(generator));
        radius = std::max(radius, 0.01);
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

chrono::ChMatrixDynamic<> ComputeJacobian(Robot *robot) {
  const size_t kNumSegs = robot->body_list.size();
  chrono::ChVectorDynamic<> theta(kNumSegs);
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
    theta(i) = angle;
    cum_theta(i) = angle;
  }
  chrono::ChVectorDynamic<> sin_cum_theta(kNumSegs);
  chrono::ChVectorDynamic<> cos_cum_theta(kNumSegs);
  for (size_t j = 0; j < kNumSegs; ++j) {
    sin_cum_theta(j) = sin(cum_theta(j));
    cos_cum_theta(j) = cos(cum_theta(j));
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
        jacobian(3 * i + 0, j) = -0.5 * l * sin_cum_theta(j);
        jacobian(3 * i + 1, j) = 0.5 * l * cos_cum_theta(j);
      } else {
        jacobian(3 * i + 0, j) =
            -l * sin_cum_theta(j) + jacobian(3 * i + 0, j + 1);
        jacobian(3 * i + 1, j) =
            l * cos_cum_theta(j) + jacobian(3 * i + 1, j + 1);
      }
      jacobian(3 * i + 2, j) = 1;
    }
  }

  jacobian.MatrTranspose();

  return jacobian;
}
