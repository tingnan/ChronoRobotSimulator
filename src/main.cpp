#include "unit_IRRLICHT/ChIrrApp.h"

#include "include/RFT.h"
#include "include/GuiControl.h"
#include "include/Robot.h"
#include "include/ChronoIO.h"
#include "include/Controller.h"

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
    if ((idA == -1 && idB != -1) || (idB == -1 && idA != -1))
      return true;
    else
      return false;
  };
};

void ApplyRFTForce(std::vector<RFTBody> &bodylist, RFTSystem &rsystem) {
  const size_t nnodes = bodylist.size();
  for (unsigned int i = 0; i < nnodes; ++i) {
    bodylist[i].GetChBody()->Empty_forces_accumulators();
    rsystem.InteractExt(bodylist[i]);
  }
}

void ParamParser(int argc, char *argv[], SnakeControlSet *params) {
  // parse info from command line
  int i = 0;
  while (i < argc) {
    switch (i) {
    case 1:
      params->k = atoi(argv[i]);
      break;
    case 2:
      params->A = atof(argv[i]);
      break;
    default:
      break;
    }
    ++i;
  }
}

int main(int argc, char *argv[]) {

  // Create a ChronoENGINE physical system
  ChSystem my_system;
  SetChronoDataPath("D:/Library/ChronoEngine/data/");
  my_system.SetIterLCPmaxItersSpeed(30);
  my_system.SetIterLCPmaxItersStab(30);
  my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SYMMSOR);
  my_system.SetTol(1e-8);
  my_system.Set_G_acc(ChVector<>(0, -9.8 * 0, 0));
  ChBroadPhaseCallbackNew *mcallback = new ChBroadPhaseCallbackNew;
  my_system.GetCollisionSystem()->SetBroadPhaseCallback(mcallback);

  // create a gui application with the chrono system
  ChIrrApp application(&my_system, L"A simple RFT example",
                       core::dimension2d<u32>(1024, 768), false, true,
                       video::EDT_OPENGL);
  ChIrrWizard::add_typical_Logo(application.GetDevice());
  ChIrrWizard::add_typical_Sky(application.GetDevice());
  ChIrrWizard::add_typical_Lights(application.GetDevice());
  const double sp = atof(argv[3]);
  ChIrrWizard::add_typical_Camera(application.GetDevice(),
                                  core::vector3df(5 * sp, 10 * sp, 0),
                                  core::vector3df(5 * sp, 0, 0));
  scene::ICameraSceneNode *cur_cam =
      application.GetSceneManager()->getActiveCamera();
  cur_cam->setRotation(irr::core::vector3df(0, 90, 0));
  GlobalControlSet simParams;
  MyEventReceiver receiver(&application, &simParams);
  application.SetUserEventReceiver(&receiver);

  //// Create a RFT ground, set the scaling factor to be 1;
  RFTSystem rsystem(&application);

  // now let us build the robot;
  ChronoRobotBuilder theSnake(&application);
  theSnake.SetRFTSystems(&rsystem);
  ParamParser(argc, argv, &simParams.snakeParams);
  receiver.UpdateText();

  theSnake.SetControlSet(&(simParams.snakeParams));
  theSnake.BuildRobot();
  theSnake.BuildBoard(sp);
  theSnake.SetCollide(false);
  application.AssetBindAll();
  application.AssetUpdateAll();

  // get all the RFT bodylist to interact
  std::vector<RFTBody> &bodylist = theSnake.getRFTBodyList();

  // set io
  ChronoIOManager ioManage(&my_system, &theSnake.getRFTBodyList());

  // get the controller
  RobotController *control = theSnake.GetController();

  // begin simulation
  application.SetStepManage(true);
  application.SetTimestep(simParams.gTimeStep);
  application.SetTryRealtime(false);

  int count = 1;
  bool switchparams = true;
  while (application.GetDevice()->run() && my_system.GetChTime() < 30) {
    // the core simulation part
    if (my_system.GetChTime() >= 1.0) {
      ApplyRFTForce(bodylist, rsystem);
    }

    application.DoStep();

    // ChVector<> cam_pos = theSnake.GetRobotCoMPosition();
    // scene::ICameraSceneNode* cur_cam =
    // application.GetSceneManager()->getActiveCamera();
    // cur_cam->setPosition(core::vector3df(cam_pos.x, cam_pos.y + 5.2,
    // cam_pos.z));
    // cur_cam->setTarget(core::vector3df(cam_pos.x, cam_pos.y, cam_pos.z));

    int savestep = 1e-2 / simParams.gTimeStep;
    // io control
    if (count == savestep) {
      control->ActiveLifting();

      application.GetVideoDriver()->beginScene(
          true, true, video::SColor(255, 140, 161, 192));
      application.DrawAll();
      if (my_system.GetChTime() >= 1.0) {
        ApplyRFTForce(bodylist, rsystem);
      }

      // draw a grid to help visualizattion
      ChIrrTools::drawGrid(
          application.GetVideoDriver(), 5, 5, 100, 100,
          ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)),
          video::SColor(255, 80, 100, 100), true);
      application.GetVideoDriver()->endScene();

      std::cout << std::fixed << std::setprecision(4) << my_system.GetChTime()
                << std::endl;
      ioManage.DumpNodInfo();
      ioManage.DumpJntInfo();
      ioManage.DumpContact();
      count = 0;
    }

    if (my_system.GetChTime() > 1.5 && switchparams) {
      // theSnake.SetCollide(true);
      switchparams = false;
    }

    if (!application.GetPaused())
      ++count;

    // screen capture?
    // application.SetVideoframeSave(true);
    // application.SetVideoframeSaveInterval(savestep);
  }

  return 0;
}
