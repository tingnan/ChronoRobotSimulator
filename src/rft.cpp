#include <algorithm>
#include <vector>
#include <array>
#include <iterator>

#include <unit_IRRLICHT/ChIrrApp.h>
#include <physics/ChBody.h>

#include "include/rft.h"

using chrono::ChSystem;
using chrono::ChVector;
using chrono::ChBody;

namespace {

const double kRFTVelocityThreshold = 1e-3;
const double kLengthThreshold = 1e-3;
const double kRoundOffError = 1e-4;

// The vertical RFT data
const double kBeta[] = { -1.5708, -1.0472, -0.5236, 0, 0.5236, 1.0472, 1.5708 };
const double kGamma[] = { -1.5708,  -1.249, -0.98279, -0.7854, -0.588,
                          -0.32175, 0,      0.32175,  0.588,   0.7854,
                          0.98279,  1.249,  1.5708 };
const double kPoppyLPForceX[] = {
  -0.0003576, 0.012707,   0.0050237,  8.9441e-05, -0.0061737, -0.0097044,
  -0.0003576, 0.022822,   0.012165,   0.00014238, -0.0062137, -0.0089274,
  -0.0084405, 0.022822,   0.028831,   0.01601,    0.00088098, -0.0055443,
  -0.010608,  0.0068018,  0.028831,   0.037173,   0.026569,   0.0070237,
  -0.0017047, -0.0051974, 0.046233,   0.037173,   0.047673,   0.025881,
  0.013093,   0.0059781,  -0.0023086, 0.053087,   0.047673,   0.066176,
  0.037016,   0.021665,   0.015063,   0.062678,   0.076347,   0.066176,
  0.07128,    0.037865,   0.025765,   0.015925,   0.089906,   0.093417,
  0.07128,    0.061707,   0.051759,   0.02385,    0.046187,   0.11504,
  0.095443,   0.061707,   0.053525,   0.038183,   0.017194,   0.04684,
  0.10661,    0.090605,   0.053525,   0.057539,   0.042789,   -0.0061161,
  0.054761,   0.10407,    0.098334,   0.057539,   0.054993,   0.037992,
  -0.034056,  0.044937,   0.099257,   0.091804,   0.054993,   0.053313,
  -0.030905,  -0.06565,   0.026716,   0.089117,   0.082993,   0.053313,
  0.002363,   -0.088024,  -0.086666,  0.0020933,  0.074291,   0.073971,
  0.002363
};
const double kPoppyLPForceY[] = {
  0.00015574, -0.012213,  -0.011769,  -0.0090943, -0.0093116, -0.0064866,
  0.00015574, -0.0074099, -0.0091358, -0.011509,  -0.011506,  -0.0015747,
  -0.0026182, -0.0074099, -0.0044121, -0.013356,  -0.012379,  -0.0058604,
  -0.0069619, 0.0011234,  -0.0044121, -0.0060136, -0.012821,  -0.015457,
  -0.011497,  -0.0086091, 0.0092621,  -0.0060136, -0.0060408, -0.011972,
  -0.012703,  -0.015659,  -0.011228,  0.011137,   -0.0060408, 0.0040482,
  -0.010925,  -0.0033929, -0.012496,  0.052096,   0.032807,   0.0040482,
  0.0094569,  -0.0076901, -0.01045,   -0.0067856, 0.094894,   0.0487,
  0.0094569,  0.010414,   0.010801,   -0.012191,  0.20682,    0.15609,
  0.050329,   0.010414,   0.013026,   0.0045585,  0.041582,   0.20268,
  0.16313,    0.060037,   0.013026,   0.026948,   0.020752,   0.13643,
  0.25041,    0.18759,    0.087413,   0.026948,   0.039794,   0.047602,
  0.20214,    0.28033,    0.21132,    0.091741,   0.039794,   0.051384,
  0.11837,    0.25197,    0.31647,    0.24544,    0.10964,    0.051384,
  0.088068,   0.15353,    0.29438,    0.35093,    0.24345,    0.11464,
  0.088068
};

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

void DrawVector(irr::ChIrrApp *ch_app, const ChVector<> &pos,
                const ChVector<> &foc, double scale, int color) {
  irr::video::IVideoDriver *driver = ch_app->GetVideoDriver();
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
  double prefac = 2.5 * 2470 * 9.81 * deltah * area * 2.5e-2;
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
  assert(dot(rft_plane_x, v_direction) <= 0);
  return rft_plane_x;
}

double GetVelAngle(const ChVector<> &v_direction,
                   const ChVector<> &y_direction) {

  double sin_gamma = dot(-v_direction, y_direction);
  return asin(sin_gamma);
}

double GetOriAngle(const ChVector<> &normal, const ChVector<> &x_direction,
                   const ChVector<> &y_direction) {
  double normal_y = dot(normal, y_direction);
  double sin_beta = dot(normal, x_direction);
  // The normal is not pointing up (along ydirection)
  if (normal_y < 0) {
    sin_beta = -sin_beta;
  }
  return asin(sin_beta);
}

template <class Scalar, class Iterator>
std::pair<Iterator, Iterator> FindLbUb(Scalar x, Iterator begin, Iterator end) {
  assert(x >= *begin && x <= *(end - 1));
  auto itr = std::lower_bound(begin, end, x);
  if (itr == begin) {
    return std::make_pair(itr, itr + 1);
  }
  return std::make_pair(itr - 1, itr);
}

size_t ColumMajorIndex2DTo1D(size_t i, size_t j, size_t n_rows) {
  return j * n_rows + i;
}

template <class Scalar, class Value>
Value Interpolate2(const Scalar xbin[], size_t size_x, const Scalar ybin[],
                   size_t size_y, const Value v[], Scalar x, Scalar y) {
  auto xbound = FindLbUb(x, xbin, xbin + size_x);
  auto ybound = FindLbUb(y, ybin, ybin + size_y);

  size_t ix0 = xbound.first - xbin;
  size_t ix1 = xbound.second - xbin;
  size_t iy0 = ybound.first - ybin;
  size_t iy1 = ybound.second - ybin;

  Value f00 = v[ColumMajorIndex2DTo1D(ix0, iy0, size_x)];
  Value f10 = v[ColumMajorIndex2DTo1D(ix1, iy0, size_x)];
  Value f01 = v[ColumMajorIndex2DTo1D(ix0, iy1, size_x)];
  Value f11 = v[ColumMajorIndex2DTo1D(ix1, iy1, size_x)];
  Value a00 = f00;
  Value a10 = f10 - f00;
  Value a01 = f01 - f00;
  Value a11 = f11 + f00 - f01 - f10;

  Scalar x_nor = (x - xbin[ix0]) / (xbin[ix1] - xbin[ix0]);
  Scalar y_nor = (y - ybin[iy0]) / (ybin[iy1] - ybin[iy0]);
  return a00 + a10 * x_nor + a01 * y_nor + a11 * x_nor * y_nor;
}
} // namespace

std::vector<chrono::ChVector<> > RFTBody::GetTransformedNormalList() {
  std::vector<chrono::ChVector<> > output;
  output.reserve(mesh.normals.size());
  for (const auto &normal : mesh.normals) {
    output.emplace_back(chbody->TransformPointLocalToParent(normal) -
                        chbody->GetPos());
  }
  return output;
}

std::vector<chrono::ChVector<> > RFTBody::GetTransformedPositionList() {
  std::vector<chrono::ChVector<> > output;
  output.reserve(mesh.positions.size());
  for (const auto &pos : mesh.positions) {
    output.emplace_back(chbody->TransformPointLocalToParent(pos));
  }
  return output;
}

std::vector<chrono::ChVector<> > RFTBody::GetTransformedVelocityList() {
  std::vector<chrono::ChVector<> > output;
  output.reserve(mesh.positions.size());
  for (const auto &pos : mesh.positions) {
    output.emplace_back(chbody->PointSpeedLocalToParent(pos));
  }
  return output;
}

RFTSystem::RFTSystem(irr::ChIrrApp *ch_app)
    : ydir_(0., 1., 0.), xdir_(1., 0., 0.), ch_app_(ch_app), ffac_(1.) {
  zdir_.Cross(xdir_, ydir_);

  /*
  ChVector<> force;
  ChVector<> pos(0, -1, 0);
  bool is_double_sided = true;
  double area = 1;
  for (double beta = -1.5; beta <= 1.5; beta += 0.05) {
    for (double gamma = -1.5; gamma <= 1.5; gamma += 0.05) {
      ChVector<> vel(2.0 * cos(gamma), 2.0 * sin(-gamma), 0);
      ChVector<> surface_normal(sin(-beta), cos(beta), 0);
      std::cout << beta << "," << gamma << " : ";
      InteractPiece(pos, vel, surface_normal, is_double_sided, area, force);
    }
  }
  */
}

void RFTSystem::InteractExt(RFTBody &rbody) {
  auto position_list = rbody.GetTransformedPositionList();
  auto velocity_list = rbody.GetTransformedVelocityList();
  auto normal_list = rbody.GetTransformedNormalList();
  const auto &area_list = rbody.mesh.areas;
  const auto &is_double_sided = rbody.mesh.is_double_sided;
  ChBody *chbody = rbody.chbody;
  // DrawVector(ch_app_, chbody->GetPos(), chbody->GetPos_dt(), 2, 1);
  if (RFTTestCollision(chbody, ydir_, 0) && chbody->GetCollide()) {
    ChVector<> force;
    ChVector<> moment;
    const size_t nn = rbody.mesh.positions.size();
    for (int i = 0; i < nn; ++i) {
      InteractPieceHorizontal(position_list[i], velocity_list[i],
                              normal_list[i], is_double_sided[i], area_list[i],
                              rbody.forces[i]);
      // DrawVector(ch_app_, position_list[i], rbody.forces[i], 50, 0);
      force += rbody.forces[i];
      ChVector<> tmp;
      tmp.Cross(position_list[i] - chbody->GetPos(), rbody.forces[i]);
      moment += tmp;
    }

    // DrawVector(ch_app_, chbody->GetPos(), force, 1, 0);
    chbody->Empty_forces_accumulators();
    chbody->Accumulate_force(force, chbody->GetPos(), false);
    chbody->Accumulate_torque(moment, false);
  }
}

void
RFTSystem::InteractPieceHorizontal(const chrono::ChVector<> &surface_position,
                                   const chrono::ChVector<> &surface_velocity,
                                   const chrono::ChVector<> &surface_normal,
                                   bool is_double_sided, double area,
                                   chrono::ChVector<> &force) {
  ChVector<> rft_plane_y = ydir_;
  double height = dot(rft_plane_y, surface_position);
  // If the normal and velocity is not in the horizontal plane
  /*
  if (fabs(surface_normal.y) > kRoundOffError ||
      fabs(surface_velocity.y) > kRoundOffError) {
    force = ChVector<>(0.0, 0.0, 0.0);
    return;
  }
  */
  if (height < 0) {
    if (dot(surface_normal, surface_velocity) < -kRoundOffError &&
        !is_double_sided) {
      force = ChVector<>(0.0, 0.0, 0.0);
      return;
    }
    double abs_vel = surface_velocity.Length();
    if (abs_vel < kRFTVelocityThreshold) {
      return;
    }
    ChVector<> v_direction = surface_velocity / abs_vel;
    // Project the velocity into x-z plane.

    // Compute the angles; We will create the norm/tangent frame first
    ChVector<> frame_normal = surface_normal;
    double cospsi = dot(v_direction, frame_normal);
    if (cospsi > 0) {
      frame_normal = -frame_normal;
    }
    cospsi = fabs(cospsi);

    ChVector<> frame_tangent;
    frame_tangent.x = -frame_normal.z;
    frame_tangent.z = frame_normal.x;
    double sinpsi = dot(frame_tangent, v_direction);
    if (sinpsi > 0) {
      frame_tangent = -frame_tangent;
    }
    sinpsi = fabs(sinpsi);
    double fnorm, fpara;
    ForceSand(-height, cospsi, sinpsi, area, &fnorm, &fpara);
    force = fnorm * frame_normal + fpara * frame_tangent;
  } else {
    force = ChVector<>(0.0, 0.0, 0.0);
  }
}

void RFTSystem::InteractPieceVertical(const ChVector<> &surface_position,
                                      const ChVector<> &surface_velocity,
                                      const ChVector<> &surface_normal,
                                      bool is_double_sided, double area,
                                      ChVector<> &force) {
  ChVector<> rft_plane_y = ydir_;
  ChVector<> rft_plane_x = xdir_;
  double height = dot(rft_plane_y, surface_position);
  if (height < 0) {
    if (dot(surface_normal, surface_velocity) < -1e-3 && !is_double_sided) {
      force = ChVector<>(0.0, 0.0, 0.0);
      return;
    }

    double abs_vel = surface_velocity.Length();
    if (abs_vel < kRFTVelocityThreshold) {
      return;
    }
    ChVector<> v_direction = surface_velocity / abs_vel;
    // The RFT vertical plane is determined by velocity and y_direction
    rft_plane_x = ComputeRFTPlaneX(v_direction, rft_plane_x, rft_plane_y);
    // If the surface normal is outside this plane, we do a projection
    ChVector<> rft_plane_normal;
    rft_plane_normal.Cross(rft_plane_x, rft_plane_y);
    double cos_proj_angle = dot(surface_normal, rft_plane_normal);

    ChVector<> projected_surface_normal =
        surface_normal - cos_proj_angle * rft_plane_normal;
    double abs_proj_normal = projected_surface_normal.Length();
    if (abs_proj_normal < kLengthThreshold) {
      force = ChVector<>(0.0, 0.0, 0.0);
      return;
    }
    projected_surface_normal /= abs_proj_normal;
    double projected_area = sqrt(1 - cos_proj_angle * cos_proj_angle) * area;
    // std::cout << surface_position << " " << surface_velocity << " : "
    //           << rft_plane_x << " " << rft_plane_y << " : ";

    // Now compute the beta and gamma defined in the paper.
    double beta =
        GetOriAngle(projected_surface_normal, rft_plane_x, rft_plane_y);
    double gamma = GetVelAngle(v_direction, rft_plane_y);
    // std::cout << beta << " " << gamma << "\n";
    double fx = Interpolate2(kBeta, 7, kGamma, 13, kPoppyLPForceX, beta, gamma);
    double fy = Interpolate2(kBeta, 7, kGamma, 13, kPoppyLPForceY, beta, gamma);
    // The force normal to RFT plane is ignored for now.
    force =
        (rft_plane_x * fx + rft_plane_y * fy) * fabs(height) * projected_area;
  } else {
    force = ChVector<>(0.0, 0.0, 0.0);
  }
}
