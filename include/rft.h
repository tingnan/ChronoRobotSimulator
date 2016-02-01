#ifndef INCLUDE_RFT_H_
#define INCLUDE_RFT_H_

#include "include/vector_utility.h"
#include <physics/ChBody.h>
#include <vector>

namespace chrono {
class ChSystem;
class ChBody;
}

namespace irr {
class ChIrrApp;
}

struct RFTMesh {
  // The position of each face in CoG frame.
  std::vector<chrono::ChVector<>> positions;
  // The orientation of each face in CoG frame.
  std::vector<chrono::ChVector<>> normals;
  // The area of each face.
  std::vector<double> areas;
  // Whether the faces are double sided.
  std::vector<bool> is_double_sided;
};

struct RFTBody {
  RFTBody(chrono::ChBody *p) : chbody(p) {}
  std::vector<chrono::ChVector<>> GetTransformedNormalList();
  std::vector<chrono::ChVector<>> GetTransformedPositionList();
  std::vector<chrono::ChVector<>> GetTransformedVelocityList();
  // The Chrono body holder.
  chrono::ChBody *chbody;
  // The mesh.
  RFTMesh mesh;
  // Force on each face in the mesh.
  std::vector<chrono::ChVector<>> forces;
};

class RFTSystem {
public:
  RFTSystem(irr::ChIrrApp *ch_app);
  void InteractExt(RFTBody &body);
  void AddHeadDrag(RFTBody &body);

private:
  void InteractPieceVertical(const chrono::ChVector<> &pos,
                             const chrono::ChVector<> &vel,
                             const chrono::ChVector<> &nor,
                             bool is_double_sided, double area,
                             chrono::ChVector<> &force);
  void InteractPieceHorizontal(const chrono::ChVector<> &pos,
                               const chrono::ChVector<> &vel,
                               const chrono::ChVector<> &nor,
                               bool is_double_sided, double area,
                               chrono::ChVector<> &force);
  chrono::ChVector<> ydir_;
  chrono::ChVector<> xdir_;
  chrono::ChVector<> zdir_;
  irr::ChIrrApp *ch_app_;
  double ffac_;
};

#endif // INCLUDE_RFT_H_
