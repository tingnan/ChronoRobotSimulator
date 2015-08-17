#ifndef INCLUDE_RFT_H_
#define INCLUDE_RFT_H_

#include <vector>

#include "include/vector_utility.h"
#include "include/mesh.h"

namespace chrono {
class ChSystem;
class ChBody;
}

namespace irr {
class ChIrrApp;
}

struct RFTBody {
  RFTBody(chrono::ChBody *p) : chbody_(p) { Resize(100); }
  void Resize(int n) {
    npiece_ = n;
    flist_.resize(n);
    plist_.resize(n);
    olist_.resize(n);
    alist_.resize(n);
    SetRFTMesh(chbody_, plist_, olist_, alist_);
  }
  int GetNumPieces() { return npiece_; }
  chrono::ChBody *GetChBody() { return chbody_; }
  // force on each of the piece
  std::vector<chrono::ChVector<> > flist_;
  // the position of each piece in CoG frame
  std::vector<chrono::ChVector<> > plist_;
  // the orientation of each piece in CoG frame
  std::vector<chrono::ChVector<> > olist_;
  // area of each of the piece;
  std::vector<double> alist_;
  size_t npiece_;
  chrono::ChBody *chbody_;
};

class RFTSystem {
public:
  RFTSystem(irr::ChIrrApp *ch_app);
  ~RFTSystem();
  void InteractExt(RFTBody &body);
  void AddHeadDrag(RFTBody &body);

private:
  void InteractPiece(const chrono::ChVector<> &pos,
                     const chrono::ChVector<> &vel,
                     const chrono::ChVector<> &ori,
                     const chrono::ChVector<> &nor,
                     const chrono::ChVector<> &fow, double area, double pdist,
                     chrono::ChVector<> &force);
  chrono::ChVector<> ydir_;
  chrono::ChVector<> xdir_;
  chrono::ChVector<> zdir_;
  irr::ChIrrApp *ch_app_;
  double ffac_;
};

#endif // INCLUDE_RFT_H_
