#include <algorithm>

#include "physics/ChBody.h"
#include "unit_IRRLICHT/ChIrrApp.h"

#include "include/RFT.h"

using namespace chrono;

RFTSystem::RFTSystem(irr::ChIrrApp *app)
    : ydir_(0., 1., 0.), xdir_(1., 0., 0.), ffac_(1.), mApp(app) {
  zdir_.Cross(xdir_, ydir_);
}

RFTSystem::~RFTSystem() {}

bool RFTTestCollision(ChBody &body, ChVector<> &pdirec, double pdist) {
  // first let us get the AABB box out of it.
  body.GetCollisionModel()->SyncPosition();
  ChVector<> bbmin, bbmax;
  body.GetCollisionModel()->GetAABB(bbmin, bbmax);
  if (dot(bbmin, pdirec) < pdist)
    return true;
  return false;
}

std::ofstream debugfile("debug.txt");
// transform the rft segments (generated from the mesher) from its body frame to
// local frame
void RFTSegsTransform(RFTBody &rbody, std::vector<ChVector<> > &pos_list,
                      std::vector<ChVector<> > &vel_list,
                      std::vector<ChVector<> > &ori_list,
                      std::vector<ChVector<> > &nor_list,
                      std::vector<ChVector<> > &for_list) {
  ChBody *chbody = rbody.GetChBody();
  if (true) {
    const int npiece = rbody.GetNumPieces();
    for (int i = 0; i < npiece; ++i) {
      pos_list[i] = chbody->TransformPointLocalToParent(rbody.plist_[i]);
      vel_list[i] = chbody->PointSpeedLocalToParent(rbody.plist_[i]);
      ori_list[i] = chbody->TransformPointLocalToParent(rbody.olist_[i]) -
                    chbody->GetPos();
      nor_list[i] = ori_list[i];
      for_list[i] = chbody->TransformPointLocalToParent(ChVector<>(1, 0, 0)) -
                    chbody->GetPos();
    }
    // debugfile << std::flush;
  }
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

void RFTSystem::InteractExt(RFTBody &rbody) {

  const int nn = rbody.GetNumPieces();
  std::vector<ChVector<> > pos_list(nn);
  std::vector<ChVector<> > vel_list(nn);
  std::vector<ChVector<> > ori_list(nn);
  std::vector<ChVector<> > nor_list(nn);
  std::vector<ChVector<> > for_list(nn);

  double poffset = 0;
  ChBody &chbody = *rbody.GetChBody();
  // std::cout << chbody.GetIdentifier() << " " << chbody.GetCollide() <<
  // std::endl;
  if (RFTTestCollision(chbody, ydir_, poffset) && chbody.GetCollide()) {
    RFTSegsTransform(rbody, pos_list, vel_list, ori_list, nor_list, for_list);
    ChVector<> force;
    ChVector<> moment;

    for (int i = 0; i < nn; ++i) {
      InteractPiece(pos_list[i], vel_list[i], ori_list[i], nor_list[i],
                    for_list[i], rbody.alist_[i], poffset, rbody.flist_[i]);
      force += rbody.flist_[i];

      /*
      if (i % 5 == 0)
      {
           DrawVector(mApp, pos_list[i], rbody.flist_[i], 0.1, 0);
           DrawVector(mApp, pos_list[i], vel_list[i], 5, 1);
      }
      */

      ChVector<> tmp;
      tmp.Cross(pos_list[i] - chbody.GetPos(), rbody.flist_[i]);
      moment += tmp;
    }
    // std::cout << force << std::endl;
    // debugfile << chbody.GetMass() << std::endl;
    DrawVector(mApp, chbody.GetPos(), force, 1, 0);
    DrawVector(mApp, chbody.GetPos(), chbody.GetPos_dt(), 2, 1);
    chbody.Accumulate_force(force, chbody.GetPos(), false);
    chbody.Accumulate_torque(moment, false);
  }
}

const double MD_RFT_VTHRESH = 1e-3;

double hevistep(double x) {
  if (x > 0)
    return 1;
  if (x < 0)
    return -1;
  if (x == 0)
    return 0;
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

//
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

void RFTSystem::InteractPiece(const ChVector<> &pos, const ChVector<> &vel,
                              const ChVector<> &ori, const ChVector<> &nor,
                              const ChVector<> &fow, double area, double pdist,
                              ChVector<> &force) {
  double height = dot(ydir_, pos);
  if (height < pdist) {
    if (dot(nor, vel) < 0) {
      force = ChVector<>(0.0, 0.0, 0.0);
      return;
    }

    double abs_vel = sqrt(dot(vel, vel));
    if (abs_vel < MD_RFT_VTHRESH) {
      return;
    }

    double cospsi = dot(vel, ori) / abs_vel;

    double sinpsi = sqrt(1 - cospsi * cospsi);
    double fnorm, fpara;
    /*
    ForceSand(pdist - height, cospsi, sinpsi, area, &fnorm, &fpara);

    ChVector<> par = fow;
    // par.x = -ori.z;
    // par.z =  ori.x;
    if (dot(par, vel) < 0)
        par = -par;
    */
    if (dot(fow, vel) < 0)
      sinpsi = -sinpsi;
    ForceHu(pdist - height, cospsi, sinpsi, area, &fnorm, &fpara);
    ChVector<> par = fow;
    // the parallel direction

    force = -fnorm * ori - fpara * par;
  } else {
    force = ChVector<>(0.0, 0.0, 0.0);
  }
}
