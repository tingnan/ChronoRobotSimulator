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

class RFTBody {
public:
  RFTBody(chrono::ChBody *p) : chbody_(p) {
    GenerateRFTMesh(chbody_, positions_, normals_, areas_);
  }

  std::vector<chrono::ChVector<> > GetTransformedNormalList();
  std::vector<chrono::ChVector<> > GetTransformedPositionList();
  std::vector<chrono::ChVector<> > GetTransformedVelocityList();
  const std::vector<double> &GetAreaList() { return areas_; }
  const std::vector<bool> &GetDoubleSided() { return is_double_sided_; }
  int GetNumPieces() { return positions_.size(); }
  chrono::ChBody *GetChBody() const { return chbody_; }
  // Force on each of the piece.
  std::vector<chrono::ChVector<> > forces;

private:
  // The position of each piece in CoG frame.
  std::vector<chrono::ChVector<> > positions_;
  // The orientation of each piece in CoG frame.
  std::vector<chrono::ChVector<> > normals_;
  // The area of each of the piece.
  std::vector<double> areas_;
  // Whether a piece is double sided.
  std::vector<bool> is_double_sided_;
  chrono::ChBody *chbody_;
};

class RFTSystem {
public:
  RFTSystem(irr::ChIrrApp *ch_app);
  ~RFTSystem();
  void InteractExt(RFTBody &body);
  void AddHeadDrag(RFTBody &body);

private:
  void InteractPieceVert(const chrono::ChVector<> &pos,
                         const chrono::ChVector<> &vel,
                         const chrono::ChVector<> &nor, bool is_double_sided,
                         double area, chrono::ChVector<> &force);
  chrono::ChVector<> ydir_;
  chrono::ChVector<> xdir_;
  chrono::ChVector<> zdir_;
  irr::ChIrrApp *ch_app_;
  double ffac_;
};

#endif // INCLUDE_RFT_H_
