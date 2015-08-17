#include <physics/ChBody.h>
#include <assets/ChAssetLevel.h>
#include <assets/ChTriangleMeshShape.h>
#include <assets/ChBoxShape.h>

#include "include/mesh.h"

using chrono::ChVector;

void MeshBox(chrono::geometry::ChBox &shape, std::vector<ChVector<> > &plist,
             std::vector<ChVector<> > &olist, std::vector<double> &alist) {
  ChVector<> half_sizes = shape.GetSize();
  double lx = 2 * half_sizes(0);
  double ly = 2 * half_sizes(1);
  double lz = 2 * half_sizes(2);
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
}

void MeshTriangle(chrono::geometry::ChTriangle &shape,
                  std::vector<ChVector<> > &plist,
                  std::vector<ChVector<> > &olist, std::vector<double> &alist) {
}

void MeshTriangleMesh(chrono::geometry::ChTriangleMeshConnected &shape,
                      std::vector<ChVector<> > &plist,
                      std::vector<ChVector<> > &olist,
                      std::vector<double> &alist) {
  const auto &face_normal_indices = shape.getIndicesNormals();
  const auto &face_normals = shape.getCoordsNormals();
  const auto &face_vertex_indices = shape.getIndicesVertexes();
  const auto &face_vertices = shape.getCoordsVertices();
  std::cout << face_normals.size() << std::endl;
  exit(0);
}

void SetRFTMesh(chrono::ChBody *body, std::vector<ChVector<> > &plist,
                std::vector<ChVector<> > &olist, std::vector<double> &alist) {
  auto &body_assets = body->GetAssets();
  for (auto &visual_asset : body_assets) {
    if (visual_asset.IsType<chrono::ChTriangleMeshShape>()) {
      auto &shape =
          visual_asset.DynamicCastTo<chrono::ChTriangleMeshShape>()->GetMesh();
      MeshTriangleMesh(shape, plist, olist, alist);
      return;
    }
    if (visual_asset.IsType<chrono::ChBoxShape>()) {
      auto &shape =
          visual_asset.DynamicCastTo<chrono::ChBoxShape>()->GetBoxGeometry();
      return;
    }
  }
}
