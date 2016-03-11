#include <algorithm>
#include <array>
#include <random>

#include <chrono_irrlicht/ChIrrApp.h>
#include <physics/ChBodyEasy.h>
#include <physics/ChSystem.h>

#include "include/controller.h"
#include "include/rft.h"
#include "include/robot.h"
#include "include/vector_utility.h"

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

Robot BuildRobot(chrono::ChSystem *ch_system, const Json::Value &params) {
  Robot i_robot;
  if (true) {
    // a set of global robotic params
    const bool kEnableVisual = true;
    const bool kEnableCollision = true;
    const double kFriction = 0.0;
    const double kDensity = 5000.0;

    std::shared_ptr<ChBodyEasyBox> ground(new ChBodyEasyBox(
        500, 1.0, 500, 1.0, !kEnableCollision, !kEnableVisual));
    ground->SetBodyFixed(true);
    ground->SetPos(ChVector<>(0, -0.5, 0));
    ground->SetIdentifier(-2);
    ground->GetMaterialSurface()->SetFriction(0.0);

    ch_system->Add(ground);
    // the Snake params
    const size_t kNumSegments = 30;
    const double kL = 1.90;
    const double kW = 0.06;
    const double kLx = kL / kNumSegments;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> z_pos_gen(-0.10, 0.10);

    ChVector<> center_pos(-kL * 1.0, -kW * 0.5, 1.0);
    std::vector<std::shared_ptr<ChBody>> body_container_;
    i_robot.inertia.resize(kNumSegments * 3, kNumSegments * 3);
    i_robot.inertia.setZero();
    for (size_t i = 0; i < kNumSegments; ++i) {
      std::shared_ptr<ChBody> body_ptr;
      if (i == kNumSegments - 1) {
        body_ptr = std::shared_ptr<ChBody>(new ChBodyEasyCylinder(
            kW * 0.5, kW, kDensity, kEnableCollision, kEnableVisual));
        i_robot.link_lengths.push_back(kW);
      } else {
        body_ptr = std::shared_ptr<ChBody>(new ChBodyEasyBox(
            kLx, kW, kW, kDensity, kEnableCollision, kEnableVisual));
        i_robot.link_lengths.push_back(kLx);
      }
      i_robot.inertia(3 * i + 0, 3 * i + 0) = body_ptr->GetMass();
      i_robot.inertia(3 * i + 1, 3 * i + 1) = body_ptr->GetMass();
      i_robot.inertia(3 * i + 2, 3 * i + 2) = body_ptr->GetInertiaXX().z;
      body_ptr->SetPos(center_pos);
      body_ptr->SetIdentifier(i);
      body_ptr->GetMaterialSurface()->SetFriction(kFriction);
      body_ptr->GetMaterialSurface()->SetCompliance(0.001f / 5.0);
      ch_system->Add(body_ptr);
      i_robot.rigid_bodies.emplace_back(body_ptr);
      i_robot.rft_bodies.emplace_back(body_ptr.get());
      // Buid the RFT body
      i_robot.rft_bodies.back().mesh = MeshRFTSquare(kLx, kW, true);
      i_robot.rft_bodies.back().forces.resize(
          i_robot.rft_bodies.back().mesh.positions.size());
      body_container_.push_back(body_ptr);
      // The engines.
      if (i > 0) {
        std::shared_ptr<ChLinkEngine> joint_ptr(new ChLinkEngine());
        ChVector<> position = center_pos - ChVector<>(0.5 * kLx, 0, 0);
        ChQuaternion<> orientation(Q_from_AngX(CH_C_PI_2));
        joint_ptr->Initialize(body_container_[i], body_container_[i - 1],
                              ChCoordsys<>(position, orientation));
        i_robot.motors.emplace_back(new ChServoMotor(joint_ptr));
        joint_ptr->SetIdentifier(i);
        ch_system->Add(joint_ptr);
      }

      if (true) {
        // The joints that maintain the snake in plane.
        std::shared_ptr<ChLinkLockPlanePlane> inplanelink(
            new ChLinkLockPlanePlane);
        inplanelink->Initialize(
            ground, body_ptr,
            ChCoordsys<>(ChVector<>(), Q_from_AngX(CH_C_PI_2)));
        ch_system->Add(inplanelink);
      }

      if (false) {
        // Add a cylinder at the joint.
        std::shared_ptr<ChBodyEasyCylinder> cylinder_ptr(new ChBodyEasyCylinder(
            kW * 0.5, kW, kDensity, kEnableCollision, kEnableVisual));
        cylinder_ptr->SetPos(center_pos + ChVector<>(kLx * 0.5, 0, 0));
        cylinder_ptr->SetIdentifier(i + 100);
        ch_system->Add(cylinder_ptr);
        std::shared_ptr<ChLinkLockLock> link(new ChLinkLockLock);
        link->Initialize(body_ptr, cylinder_ptr, ChCoordsys<>(VNULL));
        ch_system->Add(link);
      }

      center_pos += ChVector<>(kLx, 0, 0);
    }
  }

  return i_robot;
}
