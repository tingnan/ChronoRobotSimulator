#include <algorithm>
#include <vector>

#include <unit_IRRLICHT/ChIrrApp.h>
#include <physics/ChBody.h>

#include "include/rft.h"

using chrono::ChSystem;
using chrono::ChVector;
using chrono::ChBody;

namespace {

const double kRFTVelocityThreshold = 1e-3;
const double kLengthThreshold = 1e-3;

// The vertical RFT data
const double kBeta[] = {};
const double kGamma[] = {};
const double kPoppyLPForceX[] = {};
const double kPoppyLPForceZ[] = {};

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

inline double hevistep(double x) { return x > 0 ? 1 : (x < 0 ? -1 : 0); }

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

ChVector<> ComputeRFTPlaneX(const ChVector<> &v_direction,
                            const ChVector<> &x_direction,
                            const ChVector<> &y_direction) {
  ChVector<> rft_plane_normal;
  rft_plane_normal.Cross(y_direction, v_direction);
  double normal_length = length(rft_plane_normal);
  if (normal_length < kLengthThreshold) {
    return x_direction; // default
  }
  rft_plane_normal /= normal_length;
  ChVector<> rft_plane_x;
  rft_plane_x.Cross(y_direction, rft_plane_normal);
  return rft_plane_x;
}

double GetVelAngle(const ChVector<> &v_direction, const ChVector<> &x_direction,
                   const ChVector<> &y_direction) {

  double cos_gamma = dot(-v_direction, y_direction);
  if (cos_gamma > 0) {
    return acos(cos_gamma);
  } else {
    return acos(cos_gamma) - chrono::CH_C_PI;
  }
}

double GetOriAngle(const ChVector<> &ori, const ChVector<> &x_direction,
                   const ChVector<> &y_direction) {
  double beta;
  double cos_beta = dot(ori, y_direction);
  double sin_beta = dot(ori, x_direction);
  if (cos_beta < 0) {
    sin_beta = -sin_beta;
  }
  beta = asin(sin_beta);
  return beta;
}

std::pair<size_t, size_t> FindLbUb(double x, const std::vector<double> &xbin) {
  assert(x >= *xbin.begin() && x <= xbin.back());
  auto itr = std::lower_bound(xbin.begin() + 1, xbin.end() - 1, x);
  size_t index = itr - xbin.begin();
  if (*itr == x) {
    return std::make_pair(index, index);
  }
  assert(index != 0);
  return std::make_pair(index - 1, index);
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
      ChVector<> tmp;
      tmp.Cross(position_list[i] - chbody->GetPos(), rbody.forces[i]);
      moment += tmp;
    }
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
    if (dot(nor, vel) < 0 && !is_double_sided) {
      force = ChVector<>(0.0, 0.0, 0.0);
      return;
    }

    double abs_vel = sqrt(dot(vel, vel));
    if (abs_vel < kRFTVelocityThreshold) {
      return;
    }
    ChVector<> v_direction = vel / abs_vel;
    ChVector<> rft_plane_x = ComputeRFTPlaneX(v_direction, xdir_, ydir_);
    double beta = GetOriAngle(nor, rft_plane_x, ydir_);
    double gamma = GetVelAngle(v_direction, rft_plane_x, ydir_);
  } else {
    force = ChVector<>(0.0, 0.0, 0.0);
  }
}
