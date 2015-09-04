#include <array>

#include <unit_IRRLICHT/ChIrrApp.h>
#include <assets/ChColorAsset.h>
#include <physics/ChBodyEasy.h>
#include <physics/ChSystem.h>
#include "include/chfunction_squarewave.h"
#include "include/controller.h"
#include "include/robot.h"
#include "include/rft.h"
#include "json/json.h"

using namespace chrono;
using irr::ChIrrApp;

namespace {

// A Wedge is defined as a triangle (x-y plane) extruded in z direction
class Wedge {
public:
  void SetPoint(size_t index, double x, double y);
  void Recenter();
  ChVector<> GetPoint(size_t index) const;
  ChVector<> GetNormal() const;
  ChVector<> GetBaryCenter() const;
  double GetTriangleArea() const;
  double depth;

private:
  std::array<ChVector<>, 3> points_;
};

void Wedge::SetPoint(size_t index, double x, double y) {
  index = index % 3;
  points_[index](0) = x;
  points_[index](1) = y;
}

void Wedge::Recenter() {
  ChVector<> center = GetBaryCenter();
  for (size_t i = 0; i < 3; ++i) {
    points_[i] -= center;
  }
}

ChVector<> Wedge::GetPoint(size_t index) const {
  index = index % 3;
  return points_[index];
}

ChVector<> Wedge::GetNormal() const {
  auto vec1 = points_[1] - points_[0];
  auto vec2 = points_[2] - points_[1];
  ChVector<> normal;
  normal.Cross(vec1, vec2);
  if (normal(2) < 0) {
    return ChVector<>(0, 0, -1);
  }
  return ChVector<>(0, 0, 1);
}

ChVector<> Wedge::GetBaryCenter() const {
  return (points_[0] + points_[1] + points_[2]) / 3;
}

double Wedge::GetTriangleArea() const {
  ChVector<> area;
  area.Cross(points_[1] - points_[0], points_[2] - points_[1]);
  return fabs(area(2));
}

void MeshBox(ChVector<> sizes, std::vector<ChVector<> > &plist,
             std::vector<ChVector<> > &nlist, std::vector<double> &alist,
             std::vector<bool> &is_doubled_sided) {
  double lx = sizes(0);
  double ly = sizes(1);
  double lz = sizes(2);
  const size_t num_pieces = 30;
  plist.resize(num_pieces);
  nlist.resize(num_pieces);
  alist.resize(num_pieces);
  is_doubled_sided.resize(num_pieces);
  const int npiece = plist.size();
  for (int i = 0; i < npiece; ++i) {
    is_doubled_sided[i] = false;
    double denom = npiece / 2;
    if (i < denom) {
      double j = i + 0.5;
      plist[i].x = lx * j / denom - lx / 2.;
      plist[i].z = 0.5 * lz;
      nlist[i].z = 1.;
      alist[i] = lx * ly / denom;
    } else {
      double j = i + 0.5 - denom;
      plist[i].x = lx * j / denom - lx / 2.;
      plist[i].z = -0.5 * lz;
      nlist[i].z = -1.;
      alist[i] = lx * ly / denom;
    }
  }
}

void MeshWedge(const Wedge &wedge, RFTBody &rbody) {
  const size_t num_pieces = 30;
  rbody.forces.resize(num_pieces);
  rbody.positions.reserve(num_pieces);
  rbody.normals.reserve(num_pieces);
  rbody.areas.reserve(num_pieces);
  rbody.is_double_sided.reserve(num_pieces);
  const size_t np_per_side = num_pieces / 3;
  auto side_normal = wedge.GetNormal();
  for (size_t tri_idx = 0; tri_idx < 3; ++tri_idx) {
    // For each edge
    ChVector<> beg = wedge.GetPoint(tri_idx);
    ChVector<> end = wedge.GetPoint(tri_idx + 1);
    ChVector<> edge = end - beg;
    double edge_length = edge.Length();
    ChVector<> inplane_normal;
    inplane_normal.Cross(edge, side_normal);
    if (!inplane_normal.Normalize()) {
      assert(0);
    }
    for (size_t i = 0; i < np_per_side; ++i) {
      rbody.positions.emplace_back(beg + edge * double(i + 0.5) / np_per_side);
      rbody.normals.emplace_back(inplane_normal);
      rbody.areas.emplace_back(1.0 / np_per_side * edge_length * wedge.depth);
      rbody.is_double_sided.push_back(false);
    }
  }
}

ChVector<> ParseTranslation(const Json::Value &translation) {
  if (!translation.isNull()) {
    return ChVector<>(translation[0].asDouble(), translation[1].asDouble(),
                      translation[2].asDouble());
  }
  return ChVector<>();
}

ChQuaternion<> ParseRotation(const Json::Value &rotation) {
  if (!rotation.isNull()) {
    return ChQuaternion<>(rotation[0].asDouble(), rotation[1].asDouble(),
                          rotation[2].asDouble(), rotation[3].asDouble());
  }
  return ChQuaternion<>(1, 0, 0, 0);
}

ChVector<> ParsePoint(const Json::Value &point_obj) {
  ChVector<> point(point_obj[0].asDouble(), point_obj[1].asDouble(),
                   point_obj[2].asDouble());
  return point;
}

std::vector<ChVector<> > ParsePoints(const Json::Value &vertices) {
  std::vector<ChVector<> > ch_points;
  for (Json::Value::const_iterator itr = vertices.begin();
       itr != vertices.end(); ++itr) {
    ch_points.emplace_back(ParsePoint(*itr));
  }
  return ch_points;
}

ChSharedPtr<ChBody> CreateBodyFromShape(const Json::Value &shape_obj) {
  if (shape_obj.isNull()) {
    // empty body
    return ChSharedPtr<ChBody>(new ChBody);
  }
  assert(shape_obj.isMember("type"));
  auto shape_name = shape_obj["type"].asString();
  const double kDensity = 1.0;
  const bool kEnableCollision = true;
  const bool kEnableVisual = true;
  if (shape_name == "box") {
    double lx = shape_obj["lx"].asDouble();
    double ly = shape_obj["ly"].asDouble();
    double lz = shape_obj["lz"].asDouble();
    return ChSharedPtr<ChBodyEasyBox>(new ChBodyEasyBox(
        lx, ly, lz, kDensity, kEnableCollision, kEnableVisual));
  }

  if (shape_name == "convexhull") {
    auto ch_points = ParsePoints(shape_obj["vertices"]);
    return ChSharedPtr<ChBodyEasyConvexHull>(new ChBodyEasyConvexHull(
        ch_points, kDensity, kEnableCollision, kEnableVisual));
  }

  return ChSharedPtr<ChBody>(new ChBody);
}

void SetBodyMaterial(const Json::Value &material_obj, ChBody *body_ptr) {
  if (!material_obj.isNull()) {
    const double kDensity = 1.0;
    double density = material_obj.get("density", kDensity).asDouble();
    body_ptr->SetDensity(density);
    // Scale the mass/inertia with the real density.
    body_ptr->SetMass(body_ptr->GetMass() * density);
    body_ptr->SetInertia(body_ptr->GetInertia() * density);
    // Friction
    const double kFriction = 0.0;
    double friction = material_obj.get("friction", kFriction).asDouble();
    body_ptr->GetMaterialSurface()->SetFriction(friction);
  }
}

ChSharedPtr<ChBody> ParseBody(const Json::Value &body_obj) {
  // Parse the collision shape and visual shape of the body.
  const auto &shape_obj = body_obj["collision_shape"];
  ChSharedPtr<ChBody> body_ptr = CreateBodyFromShape(shape_obj);
  body_ptr->SetBodyFixed(body_obj.get("is_dynamic", false).asBool());
  // Parse the transformation of the body.
  const auto &frame_obj = body_obj["frame"];
  body_ptr->SetPos(ParseTranslation(frame_obj["translation"]));
  body_ptr->SetRot(ParseRotation(frame_obj["rotation"]));
  // Parse the material property of the body.
  const auto &material_obj = body_obj["material"];
  SetBodyMaterial(material_obj, body_ptr.get());
  return body_ptr;
}

} // namespace

WorldBuilder::WorldBuilder(ChIrrApp *pApp) : app_(pApp) {}

void WorldBuilder::CreateRigidBodies(const Json::Value &body_list) {
  ChSystem *ch_system = app_->GetSystem();
  for (Json::Value::const_iterator itr = body_list.begin();
       itr != body_list.end(); ++itr) {
    ChSharedPtr<ChBody> body_ptr = ParseBody(*itr);
    ch_system->Add(body_ptr);
  }

  app_->AssetBindAll();
  app_->AssetUpdateAll();
  /* {
    ChSharedBodyPtr ground(new ChBodyEasyBox(100, 1, 100, 1, false, false));
    ground->SetBodyFixed(true);
    ch_system->Add(ground);

    // Create a wedge shape.
    double lx = 5.0;
    double ly = 3.0;
    double lz = 5.0;

    std::vector<ChVector<> > points;
    Wedge wedge;
    wedge.SetPoint(0, lx * 0.5, 0);
    wedge.SetPoint(1, -lx * 0.5, 0);
    wedge.SetPoint(2, 0, ly);
    wedge.depth = lz;
    wedge.Recenter();
    for (int sign = -1; sign <= 1; sign += 2) {
      for (size_t i = 0; i < 3; ++i) {
        ChVector<> pt = wedge.GetPoint(i);
        pt(2) = sign * lz * 0.5;
        points.emplace_back(pt);
      }
    }
    ChSharedBodyPtr foot(new ChBodyEasyConvexHull(points, 1, true, true));
    foot->SetPos(ChVector<>(0, -depth, 0));
    foot->SetRot(Q_from_AngZ(beta));
    ch_system->Add(foot);
    // Create the rft body of the foot
    rft_body_list_.emplace_back(foot.get());
    RFTBody &rbody = rft_body_list_.back();
    MeshWedge(wedge, rbody);
    // Now Add linear actuator.

    ChVector<> origin(0, 0, 0);
    ChQuaternion<> orientation = Q_from_AngX(CH_C_PI_2) * Q_from_AngY(gamma);
    ChVector<> z_direction = orientation.Rotate(ChVector<>(0, 0, 1));

    ChSharedPtr<ChLinkLockPrismatic> prismatic(new ChLinkLockPrismatic());
    prismatic->Initialize(foot, ground, ChCoordsys<>(origin, orientation));
    ch_system->Add(prismatic);

    ChSharedPtr<ChLinkLinActuator> actuator(new ChLinkLinActuator());
    actuator->Initialize(
        foot, ground, false, ChCoordsys<>(foot->GetPos(), QUNIT),
        ChCoordsys<>(foot->GetPos() + 100.0 * z_direction, QUNIT));
    actuator->Set_lin_offset(100);
    ch_system->Add(actuator);
    controller_.AddEngine(actuator.get());
  }*/
}
