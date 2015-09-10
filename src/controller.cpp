#include <iostream>
#include <string>
#include <sstream>

#include <physics/ChSystem.h>

#include "include/chfunction_pid.h"
#include "include/controller.h"
#include "include/robot.h"

using namespace chrono;

void RobotController::SetControlStrategy() {
  for (int i = 0; i < engine_list_.size(); ++i) {
    ChSharedPtr<ChFunction_Sine> funct(new ChFunction_Sine(0, 0.2, 10));
    if (ChLinkEngine *engine = dynamic_cast<ChLinkEngine *>(engine_list_[i])) {
      engine->Set_rot_funct(funct);
    }
  }
}
