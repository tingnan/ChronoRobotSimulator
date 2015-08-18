#include <algorithm>

#include <unit_IRRLICHT/ChIrrApp.h>
#include <physics/ChBody.h>

#include "include/rft.h"

using chrono::ChSystem;
using chrono::ChVector;
using chrono::ChBody;

namespace {

bool RFTTestCollision(ChBody *body, const ChVector<> &pdirec, double pdist) {
  // first let us get the AABB box out of it.
  chrono::collision::ChCollisionModel *collision_model =
      body->GetCollisionModel();
  collision_model->SyncPosition();

  ChVector<> bbmin, bbmax;
  collision_model->GetAABB(bbmin, bbmax);
  if (dot(bbmin, pdirec) < pdist)
    return true;
  return false;
}

void DrawVector(irr::ChIrrApp *mApp, const ChVector<> &pos,
                const ChVector<> &foc, double scale, int color) {
  irr::video::IVideoDriver *driver = mApp->GetVideoDriver();
  driver->setTransform(irr::video::ETS_WORLD, irr::core::matrix4());
  irr::video::SColor mcol;
  if (color == 0)
    mcol = irr::video::SColor(70, 125, 0, 0);
  if (color == 1)
    mcol = irr::video::SColor(70, 0, 50, 255);
  driver->draw3DLine(irr::core::vector3dfCH(pos),
                     irr::core::vector3dfCH(pos + foc * scale), mcol);
}

const double MD_RFT_VTHRESH = 1e-3;

double hevistep(double x) {
  if (x > 0)
    return 1;
  if (x < 0)
    return -1;
  return 0;
}

void ForceSand(double deltah, double cospsi, double sinpsi, double area,
               double *fnorm, double *fpara) {
  // prefac = k * rho * g * z * A * reduction_factor
  double prefac = 2.5 * 2470 * 9.81 * deltah * area * 1e-3;
  *fpara = prefac * sinpsi * 1;
  const double tan2gamm0 = 0.060330932472924;
  *fnorm = prefac * cospsi * (1 + 1.8 / sqrt(tan2gamm0 + cospsi * cospsi));
}

void ForceBB(double deltah, double cospsi, double sinpsi, double area,
             double *fnorm, double *fpara) {
  double prefac = area * deltah / (0.00112 * 0.076);
  double CS = 3.21;
  double gamma = 2.79;
  double CF = 1.34;
  double CL = -0.82;
  *fnorm =
      prefac * CS * gamma * cospsi / sqrt(1 + gamma * gamma * cospsi * cospsi);
  *fpara = 1 * prefac * (CF * sinpsi + CL * (1 - cospsi));
}

void ForceHu(double deltah, double cospsi, double sinpsi, double area,
             double *fnorm, double *fpara) {
  // cospsi (n \cdot v), sinpsi (t \cdot v)
  // force is proportional to velocity as well
  double prefac = 9.81 * deltah * area * 40;
  double mu_t = 0.30, mu_f = 0.11, mu_b = 0.14;
  *fnorm = prefac * mu_t * cospsi;
  *fpara = prefac * (mu_f * hevistep(sinpsi) + mu_b * (1 - hevistep(sinpsi))) *
           sinpsi;
}
}

std::vector<chrono::ChVector<> > RFTBody::GetTransformedNormalList() {
  std::vector<chrono::ChVector<> > output;
  output.reserve(normals_.size());
  for (const auto &normal : normals_) {
    output.emplace_back(chbody_->TransformPointLocalToParent(normal) -
                        chbody_->GetPos());
  }
  return output;
}

std::vector<chrono::ChVector<> > RFTBody::GetTransformedPositionList() {
  std::vector<chrono::ChVector<> > output;
  output.reserve(positions_.size());
  for (const auto &pos : positions_) {
    output.emplace_back(chbody_->TransformPointLocalToParent(pos));
  }
  return output;
}

std::vector<chrono::ChVector<> > RFTBody::GetTransformedVelocityList() {
  std::vector<chrono::ChVector<> > output;
  output.reserve(positions_.size());
  for (const auto &pos : positions_) {
    output.emplace_back(chbody_->PointSpeedLocalToParent(pos));
  }
  return output;
}

RFTSystem::RFTSystem(irr::ChIrrApp *ch_app)
    : ydir_(0., 1., 0.), xdir_(1., 0., 0.), ch_app_(ch_app), ffac_(1.) {
  zdir_.Cross(xdir_, ydir_);
}

RFTSystem::~RFTSystem() {}

void RFTSystem::InteractExt(RFTBody &rbody) {

  auto position_list = rbody.GetTransformedPositionList();
  auto velocity_list = rbody.GetTransformedVelocityList();
  auto normal_list = rbody.GetTransformedNormalList();
  const auto &area_list = rbody.GetAreaList();
  const auto &is_double_sided = rbody.GetDoubleSided();

  ChBody *chbody = rbody.GetChBody();
  if (RFTTestCollision(chbody, ydir_, 0) && chbody->GetCollide()) {
    ChVector<> force;
    ChVector<> moment;
    const size_t nn = rbody.GetNumPieces();
    for (int i = 0; i < nn; ++i) {
      InteractPiece(position_list[i], velocity_list[i], normal_list[i],
                    is_double_sided[i], area_list[i], rbody.forces[i]);
      force += rbody.forces[i];

      /*
      if (i % 5 == 0)
      {
           DrawVector(mApp, pos_list[i], rbody.flist_[i], 0.1, 0);
           DrawVector(mApp, pos_list[i], vel_list[i], 5, 1);
      }
      */

      ChVector<> tmp;
      tmp.Cross(position_list[i] - chbody->GetPos(), rbody.forces[i]);
      moment += tmp;
    }
    // std::cout << force << std::endl;
    // debugfile << chbody.GetMass() << std::endl;
    DrawVector(ch_app_, chbody->GetPos(), force, 1, 0);
    DrawVector(ch_app_, chbody->GetPos(), chbody->GetPos_dt(), 2, 1);
    chbody->Accumulate_force(force, chbody->GetPos(), false);
    chbody->Accumulate_torque(moment, false);
  }
}

void RFTSystem::InteractPiece(const ChVector<> &pos, const ChVector<> &vel,
                              const ChVector<> &nor, bool is_double_sided,
                              double area, ChVector<> &force) {
  double height = dot(ydir_, pos);
  if (height < 0) {
    if (dot(nor, vel) < 0) {
      force = ChVector<>(0.0, 0.0, 0.0);
      return;
    }

    double abs_vel = sqrt(dot(vel, vel));
    if (abs_vel < MD_RFT_VTHRESH) {
      return;
    }
  } else {
    force = ChVector<>(0.0, 0.0, 0.0);
  }
}
