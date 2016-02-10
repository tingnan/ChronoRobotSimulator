#include <chrono_irrlicht/ChIrrApp.h>
#include <motion_functions/ChFunction_Sine.h>

#include "include/chrono_io.h"
#include "include/controller.h"
#include "include/gui.h"
#include "include/rft.h"
#include "include/robot.h"
#include "include/world.h"
#include "json/json.h"

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
    if (abs(idA - idB) > 1) {
      return true;
    }
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
  auto &rigid_bodies = robot->rigid_bodies;
  std::vector<RFTBody> rft_bodies;
  for (auto &rigid_body : rigid_bodies) {
    rft_bodies.emplace_back(rigid_body.get());
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
  ch_system.Set_G_acc(ChVector<>(0, 0, 0));
  ChBroadPhaseCallbackNew *mcallback = new ChBroadPhaseCallbackNew;
  ch_system.GetCollisionSystem()->SetBroadPhaseCallback(mcallback);

  // create a gui ch_app with the chrono system
  ChIrrApp ch_app(&ch_system, L"snake", core::dimension2d<u32>(800, 800), false,
                  true, video::EDT_OPENGL);
  ChIrrWizard::add_typical_Logo(ch_app.GetDevice());
  ChIrrWizard::add_typical_Sky(ch_app.GetDevice());
  ChIrrWizard::add_typical_Lights(ch_app.GetDevice());
  ChIrrWizard::add_typical_Camera(ch_app.GetDevice(),
                                  core::vector3df(1.4, 5, 0.0),
                                  core::vector3df(1.5, 0, 0.0));
  scene::ICameraSceneNode *cur_cam =
      ch_app.GetSceneManager()->getActiveCamera();
  //// Create a RFT ground, set the scaling factor to be 1;
  RFTSystem rsystem(&ch_app);

  // now let us build the robot_builder;
  Json::Value lattice_params;
  if (argc < 2) {
    std::cout << "no lattice params input!\n";
    exit(0);
  }
  lattice_params["spacing"] = atof(argv[1]);
  BuildWorld(ch_app.GetSystem(), lattice_params);
  Robot i_robot = BuildRobot(ch_app.GetSystem(), Json::Value());

  // Do visual binding.
  ch_app.AssetBindAll();
  ch_app.AssetUpdateAll();

  // get all the RFT body_list to interact

  // set io
  std::ofstream mov_file("mov.dat");
  std::ofstream jnt_file("jnt.dat");
  std::ofstream rft_file("rft.dat");
  std::ofstream cot_file("cot.dat");

  // begin simulation

  int count = 0;
  int save_step = 4e-2 / ch_app.GetTimestep();

  // Use a controller
  Controller controller(&ch_system, &i_robot);

  // Now set the controller params.
  if (argc < 4) {
    std::cout << "no command params input!\n";
    exit(0);
  }
  Json::Value command_params;
  command_params["duration"] = atof(argv[2]);
  command_params["amplitude"] = atof(argv[3]);
  controller.SetDefaultParams(command_params);
  controller.EnablePosMotorControl();

  // Even receiver
  MyEventReceiver receiver(&ch_app, &controller);
  ch_app.SetUserEventReceiver(&receiver);

  // screen capture?
  ch_app.SetVideoframeSave(false);
  ch_app.SetVideoframeSaveInterval(save_step);

  while (ch_app.GetDevice()->run() && ch_system.GetChTime() < 50) {
    // the core simulation part
    controller.Step(ch_app.GetTimestep());
    ch_app.DoStep();

    // io control
    if (count == save_step - 1) {
      ch_app.GetVideoDriver()->beginScene(true, true,
                                          video::SColor(255, 140, 161, 192));
      ch_app.DrawAll();
      ApplyRFTForce(i_robot.rft_bodies, rsystem);
      // draw a grid to help visualizattion
      ChIrrTools::drawGrid(
          ch_app.GetVideoDriver(), 5, 5, 100, 100,
          ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)),
          video::SColor(255, 80, 100, 100), true);
      ch_app.GetVideoDriver()->endScene();

      if (!ch_app.GetPaused()) {
        // std::cout << std::fixed << std::setprecision(4) <<
        // ch_system.GetChTime()
        //          << std::endl;
        if (ch_system.GetChTime() > 20) {
        }
      }
      count = 0;
      continue;
    }

    ApplyRFTForce(i_robot.rft_bodies, rsystem);
    if (!ch_app.GetPaused())
      ++count;
  }

  return 0;
}
