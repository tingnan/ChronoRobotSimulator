#ifndef INCLUDE_MESHER_H_
#define INCLUDE_MESHER_H_
#include <vector>

#include <core/ChVector.h>

namespace chrono {
class ChBody;
} // namespace chrono

void GenerateRFTMesh(chrono::ChBody *pbody, std::vector<chrono::ChVector<> > &,
                     std::vector<chrono::ChVector<> > &, std::vector<double> &);
#endif // INCLUDE_MESHER_H_
