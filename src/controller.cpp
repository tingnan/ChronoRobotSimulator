#include <iostream>
#include <string>
#include <sstream>

#include <physics/ChSystem.h>

#include "include/chfunction_squarewave.h"
#include "include/chfunction_pid.h"
#include "include/chfunction_data.h"
#include "include/controller.h"
#include "include/robot.h"

using namespace chrono;

namespace {
void ReadCSV(std::ifstream &inputfile, std::vector<double> &time,
             std::vector<double> &data) {
  std::string str;
  int i = 0;
  while (std::getline(inputfile, str, '\n')) {
    std::istringstream myline(str);
    std::string mynum;
    int j = 0;
    std::vector<double> tmpline(256);
    while (std::getline(myline, mynum, ',')) {
      tmpline[j] = strtod(mynum.c_str(), NULL);
      ++j;
    }
    time.push_back(tmpline[0]);
    data.push_back(tmpline[1]);
    ++i;
  }
}
} // namespace

RobotController::RobotController() {}

void RobotController::EnablePositionControl() {
  for (int i = 0; i < motors_.size(); ++i) {
    ChSharedPtr<ChFunction_Sine> funct(new ChFunction_Sine(0, 0.2, 10));
    if (ChLinkLinActuator *mylink =
            dynamic_cast<ChLinkLinActuator *>(motors_[i])) {
      mylink->Set_dist_funct(funct);
    }
    if (ChLinkEngine *mylink = dynamic_cast<ChLinkEngine *>(motors_[i])) {
      mylink->Set_rot_funct(funct);
    }
  }
}

void RobotController::EnableActiveLifting() {}

void RobotController::EnableTorqueControl() {}
