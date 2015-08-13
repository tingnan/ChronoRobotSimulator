#include "physics/ChBody.h"
#include "collision/ChCModelBullet.h"
#include "btBulletCollisionCommon.h"

#include "include/Mesher.h"

using chrono::ChVector;

void MeshBox(const btBoxShape *cshape, double envelope,
             std::vector<ChVector<> > &plist, std::vector<ChVector<> > &olist,
             std::vector<double> &alist) {
  btVector3 halfExtents = cshape->getHalfExtentsWithMargin();
  double lx = 2 * (halfExtents.getX() - envelope);
  double ly = 2 * (halfExtents.getY() - envelope);
  double lz = 2 * (halfExtents.getZ() - envelope);
  const int npiece = plist.size();
  for (int i = 0; i < npiece; ++i) {
    double denom = npiece / 2;
    if (i < denom) {
      double j = i + 0.5;
      plist[i].x = lx * j / denom - lx / 2.;
      plist[i].z = 0.5 * lz;
      olist[i].z = 1.;
      alist[i] = lx * ly / denom;
    } else {
      double j = i + 0.5 - denom;
      plist[i].x = lx * j / denom - lx / 2.;
      plist[i].z = -0.5 * lz;
      olist[i].z = -1.;
      alist[i] = lx * ly / denom;
    }
  }
  // std::cout << lx << " " << ly << " " << lz << std::endl;
}

void SetRFTMesh(chrono::ChBody *pbody, std::vector<ChVector<> > &plist,
                std::vector<ChVector<> > &olist, std::vector<double> &alist) {
  chrono::collision::ChModelBullet *cbody =
      (chrono::collision::ChModelBullet *)(pbody->GetCollisionModel());
  btCollisionShape *cshape = cbody->GetBulletModel()->getCollisionShape();
  if (strcmp(cbody->GetBulletModel()->getCollisionShape()->getName(), "Box") ==
      0) {
    MeshBox((btBoxShape *)cshape, cbody->GetEnvelope(), plist, olist, alist);
  }
}
