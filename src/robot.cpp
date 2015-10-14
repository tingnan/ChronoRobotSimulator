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
  Robot i_robot;
  auto ch_system = ch_app->GetSystem();
  const double kDensity = 2700.0;
  const bool kEnableCollision = true;
  const bool kEnableVisual = true;
  const double lfact = 1e2;
  const double rhofact = 1e3;
  ChSharedPtr<ChBodyEasyBox> ground(new ChBodyEasyBox(
      20, 0.05, 20, kDensity, kEnableCollision, kEnableVisual));
  const double bodyheight = 30. / lfact;
  ChSharedPtr<ChColorAsset> ground_color(new ChColorAsset);
  ground_color->SetColor(ChColor(0.3, 0.25, 0.15, 0.3));
  ground->AddAsset(ground_color);
  // ChSharedPtr<ChColorAsset> mcolor(new ChColorAsset);
  // mcolor->SetColor(ChColor(0.2f, 0.8f, 0.2f));
  // ground->AddAsset(mcolor);

  ground->SetPos(ChVector<>(0, 0, 0.));
  ground->SetBodyFixed(true);
  ground->SetId(-1);
  ground->SetIdentifier(-1);
  ch_system->AddBody(ground);

  // create a torque controlled robot
  {
    ChSharedPtr<ChBodyEasyBox> upperframe(new ChBodyEasyBox(
        2. / lfact, 2. / lfact, 15. / lfact, 7. * rhofact, true));
    ch_system->AddBody(upperframe);
    upperframe->SetPos(ChVector<>(0, bodyheight, 0.));
    upperframe->SetId(0);
    upperframe->SetBodyFixed(true);
    ChSharedPtr<ChTexture> leg_texture(new ChTexture);
    leg_texture->SetTextureFilename(
        "/usr/local/chrono/data/cubetexture_borders.png");
    upperframe->AddAsset(leg_texture);
    // upperframe->SetBodyFixed(true);
    ChSharedPtr<ChLinkLockOldham> inplanelink(new ChLinkLockOldham);
    inplanelink->Initialize(ground, upperframe, ChCoordsys<>(ChVector<>()));
    ch_system->AddLink(inplanelink);

    const double leglen[4] = { 7.57 / lfact, 7.77 / lfact,
                               7.48 / lfact, 3.40 / lfact };
    const double jnt_angles[] = { 0.9647466202,  -1.7087647716, 1.3293575652,
                                  -0.3548684919, -0.7016915259, -1.6153386499,
                                  1.2546486350,  -0.3591937412 };
    const ChQuaternion<> qter = Q_from_AngZ(-CH_C_PI_2);
    std::vector<ChSharedPtr<ChBody> > legcontainer(8);

    for (int j = 0; j < 2; ++j) {
      double z_step = -7.5 / lfact;
      if (j == 1)
        z_step = -z_step;
      double y_step = bodyheight + leglen[0] / 2.;
      for (int k = 0; k < 4; ++k) {

        double x_step = 0. / lfact;
        if (k == 0)
          y_step = y_step - leglen[k];
        else
          y_step = y_step - 0.5 * (leglen[k] + leglen[k - 1]);
        double legw = 2.0 / lfact;
        double legh = 2.0 / lfact;
        if (k == 3) {
          legw = 8.0 / lfact;
          std::vector<ChVector<> > points;
          for (int lidx = -1; lidx <= 1; lidx += 2) {
            double z_shift = legw * 0.5 * lidx;
            points.emplace_back(0, legw * 0.5, z_shift);
            points.emplace_back(0, -legw * 0.5, z_shift);
            points.emplace_back(-leglen[k], 0, z_shift);
          }
          legcontainer[j * 4 + k] = ChSharedPtr<ChBodyEasyConvexHull>(
              new ChBodyEasyConvexHull(points, 5. * rhofact, true));
        } else {
          legcontainer[j * 4 + k] = ChSharedPtr<ChBodyEasyBox>(
              new ChBodyEasyBox(leglen[k], legw, legw, 5. * rhofact, true));
        }
        ChSharedPtr<ChTexture> leg_texture(new ChTexture);
        leg_texture->SetTextureFilename(
            "/usr/local/chrono/data/cubetexture_borders.png");
        legcontainer[j * 4 + k]->AddAsset(leg_texture);
        legcontainer[j * 4 + k]->SetRot(qter);
        legcontainer[j * 4 + k]->SetPos(ChVector<>(x_step, y_step, z_step));
        legcontainer[j * 4 + k]->SetId(j * 4 + k + 1);
        ch_system->AddBody(legcontainer[j * 4 + k]);

        ChSharedPtr<ChLinkEngine> mylink(new ChLinkEngine);
        if (k == 0)
          mylink->Initialize(legcontainer[j * 4 + k], upperframe,
                             ChCoordsys<>(ChVector<>(
                                 x_step, y_step + leglen[k] / 2.0, z_step)));
        else
          mylink->Initialize(legcontainer[j * 4 + k],
                             legcontainer[j * 4 + k - 1],
                             ChCoordsys<>(ChVector<>(
                                 x_step, y_step + leglen[k] / 2.0, z_step)));
        mylink->Set_eng_mode(ChLinkEngine::ENG_MODE_ROTATION);
        ChSharedPtr<ChFunction_Const> joint_function(
            new ChFunction_Const(jnt_angles[j * 4 + k]));
        // ChFunction_Data *funptr = new ChFunction_Data(data);
        // funptr->SetColumn(j * 4 + k + 1);
        // mylink->Set_rot_funct(funptr);
        mylink->Set_rot_funct(joint_function);
        ch_system->AddLink(mylink);
      }
    }
  }

  // Do visual binding.
  ch_app->AssetBindAll();
  ch_app->AssetUpdateAll();

  return i_robot;
}
