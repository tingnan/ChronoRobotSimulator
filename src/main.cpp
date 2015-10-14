#include <chrono_irrlicht/ChIrrApp.h>
#include <motion_functions/ChFunction_Sine.h>

#include "json/json.h"
#include "include/rft.h"
#include "include/gui.h"
#include "include/robot.h"
#include "include/controller.h"
#include "include/chrono_io.h"

namespace {
std::string Stringify(const char *path) {
  std::ifstream file_handle(path, std::ios::binary | std::ios::in);
  std::string text;
  if (file_handle.is_open()) {
    file_handle.seekg(0, std::ios::end);
    text.resize(file_handle.tellg());
    file_handle.seekg(0, std::ios::beg);
    file_handle.read(&text[0], text.size());
    file_handle.close();
  } else {
    std::cout << "cannot open file\n";
  }
  return text;
}

Json::Value CreateJsonObject(const char *file) {
  std::string content = Stringify(file);
  Json::Reader reader;
  Json::Value json_obj;
  bool success = reader.parse(content, json_obj);
  if (!success) {
    std::cout << "not a valid json file" << std::endl;
    exit(-1);
  }
  return json_obj;
}
} // namespace

using namespace chrono;
using namespace irr;

class ChBroadPhaseCallbackNew : public collision::ChBroadPhaseCallback {
public:
  /// Callback used to report 'near enough' pairs of models.
  /// This must be implemented by a child class of ChBroadPhaseCallback.
  /// Return false to skip narrow-phase contact generation for this pair of
  /// bodies.
  bool BroadCallback(collision::ChCollisionModel *mmodelA, ///< pass 1st model
                     collision::ChCollisionModel *mmodelB  ///< pass 2nd model
                     ) {
    int idA = mmodelA->GetPhysicsItem()->GetIdentifier();
    int idB = mmodelB->GetPhysicsItem()->GetIdentifier();
    if (idA == -1 || idB == -1 || idA == -2 || idB == -2)
      return true;
    return false;
  };
};

void ApplyRFTForce(std::vector<RFTBody> &rft_body_list, RFTSystem &rsystem) {
  const size_t kNumBodies = rft_body_list.size();
  for (unsigned int i = 0; i < kNumBodies; ++i) {
    rsystem.InteractExt(rft_body_list[i]);
  }
}

std::vector<RFTBody> BuildRFTBody(Robot *robot) {
  auto &body_list = robot->body_list;
  std::vector<RFTBody> rft_body_list;
  for (size_t i = 0; i < body_list.size(); ++i) {
    rft_body_list.emplace_back(body_list[i]);
  }
}

int main(int argc, char *argv[]) {

  // Create a ChronoENGINE physical system
  ChSystem ch_system;
  SetChronoDataPath("/usr/local/chrono/data/");
  ch_system.SetIterLCPmaxItersSpeed(30);
  ch_system.SetIterLCPmaxItersStab(30);
  // ch_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SYMMSOR);
  ch_system.SetTol(1e-8);
  ch_system.Set_G_acc(ChVector<>(0, -9.8, 0));
  ChBroadPhaseCallbackNew *mcallback = new ChBroadPhaseCallbackNew;
  ch_system.GetCollisionSystem()->SetBroadPhaseCallback(mcallback);

  // create a gui ch_app with the chrono system
  ChIrrApp ch_app(&ch_system, L"A simple RFT example",
                  core::dimension2d<u32>(600, 600), false, true,
                  video::EDT_OPENGL);
  ChIrrWizard::add_typical_Logo(ch_app.GetDevice());
  ChIrrWizard::add_typical_Sky(ch_app.GetDevice());
  ChIrrWizard::add_typical_Lights(ch_app.GetDevice());
  ChIrrWizard::add_typical_Camera(ch_app.GetDevice(),
                                  core::vector3df(0.5, 0.5, -0.5),
                                  core::vector3df(0.1, 0.2, -0.1));
  scene::ICameraSceneNode *cur_cam =
      ch_app.GetSceneManager()->getActiveCamera();
  // cur_cam->setRotation(irr::core::vector3df(0, 90, 0));
  MyEventReceiver receiver(&ch_app);
  ch_app.SetUserEventReceiver(&receiver);

  //// Create a RFT ground, set the scaling factor to be 1;
  RFTSystem rsystem(&ch_app);
  // now let us build the robot_builder;

  Robot i_robot = BuildRobotAndWorld(&ch_app, Json::Value());
  // UsePositionControl(&i_robot);
  // get all the RFT body_list to interact
  // std::vector<RFTBody> &body_list = robot_builder.getRFTBodyList();

  // set io
  std::ofstream mov_file("mov.dat");
  std::ofstream jnt_file("jnt.dat");
  std::ofstream rft_file("rft.dat");

  // begin simulation

  int count = 0;
  int save_step = 1e-1 / ch_app.GetTimestep();
  // screen capture?

  // Assemble the robot

  while (ch_system.GetChTime() < 0.0) {
    ch_app.DoStep();
    std::cout << std::fixed << std::setprecision(4) << ch_system.GetChTime()
              << std::endl;
  }

  // Switch to controller
  Controller controller(&ch_system, &i_robot);
  // UseController(&controller);

  ch_app.SetVideoframeSave(true);
  ch_app.SetVideoframeSaveInterval(save_step);

  while (ch_app.GetDevice()->run() && ch_system.GetChTime() <= 100.0) {
    // the core simulation part
    controller.Step(ch_app.GetTimestep());
    ch_app.DoStep();

    // ChVector<> cam_pos = robot_builder.GetRobotCoMPosition();
    // scene::ICameraSceneNode* cur_cam =
    // ch_app.GetSceneManager()->getActiveCamera();
    // cur_cam->setPosition(core::vector3df(cam_pos.x, cam_pos.y + 5.2,
    // cam_pos.z));
    // cur_cam->setTarget(core::vector3df(cam_pos.x, cam_pos.y, cam_pos.z));

    // io control
    if (count == save_step - 1) {
      ch_app.GetVideoDriver()->beginScene(true, true,
                                          video::SColor(255, 140, 161, 192));
      ch_app.DrawAll();
      ApplyRFTForce(i_robot.rft_body_list, rsystem);
      // draw a grid to help visualizattion
      ChIrrTools::drawGrid(
          ch_app.GetVideoDriver(), 5, 5, 100, 100,
          ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)),
          video::SColor(255, 80, 100, 100), true);
      ch_app.GetVideoDriver()->endScene();

      if (!ch_app.GetPaused()) {
        std::cout << std::fixed << std::setprecision(4) << ch_system.GetChTime()
                  << std::endl;
        SerializeBodies(i_robot.body_list, mov_file);
        SerializeEngines(i_robot.engine_list, jnt_file);
      }
      count = 0;
      continue;
    }

    ApplyRFTForce(i_robot.rft_body_list, rsystem);
    if (!ch_app.GetPaused())
      ++count;
  }

  return 0;
}
