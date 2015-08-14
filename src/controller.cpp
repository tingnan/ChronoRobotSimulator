#include <iostream>
#include <string>
#include <sstream>

#include <physics/ChLinkEngine.h>

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
  /*
  double kk = robot_params_->k;
  double AA = robot_params_->A;
  double ww = robot_params_->w;

  for (int i = 0; i < motors_.size(); ++i) {
    ChLinkEngine *mylink = motors_[i];
    mylink->Set_eng_mode(ChLinkEngine::ENG_MODE_ROTATION);
    ChSharedPtr<ChFunction_Sine> rotfunc(
        new ChFunction_Sine(double(i * 2) / kk * CH_C_2PI, ww, AA));
    mylink->Set_rot_funct(rotfunc);
  }
  */
}

void RobotController::EnableActiveLifting() {
  /*
  for (int i = 0; i < motors_.size(); ++i) {
    // get the current angle of bending
    ChLinkEngine *mylink = motors_[i];
    ChBody *b1 = mylink->GetMarker1()->GetBody();
    ChBody *b2 = mylink->GetMarker2()->GetBody();
    double angle = fabs(mylink->Get_mot_rot());
    // std::cout << angle << std::endl;
    if (angle > 0.4) {
      // disable the collision on the links
      b1->SetCollide(false);
      b2->SetCollide(false);
    } else {
      b1->SetCollide(true);
      b2->SetCollide(true);
    }
    // std::cout << b1->GetIdentifier() << " " << b1->GetCollide() << std::endl;
    // std::cout << b2->GetIdentifier() << " " << b2->GetCollide() << std::endl;
    // std::cout << id1 << " " << id2 << " " << mylink->Get_mot_rot() <<
    // std::endl;
  }
  */
}

void RobotController::EnableTorqueControl() {
  /*
  double kk = robot_params_->k;
  double AA = robot_params_->A;
  double ww = robot_params_->w;

  for (int i = 0; i < motors_.size(); ++i) {
    ChLinkEngine *mylink = motors_[i];
    // double torquelimit = 1.1;
    mylink->Set_eng_mode(ChLinkEngine::ENG_MODE_TORQUE);
    ChSharedPtr<ChFunction_Sine> rotfunc(
        new ChFunction_Sine(double(i * 2) / kk * CH_C_2PI, ww, AA));
    mylink->Set_rot_funct(rotfunc);
    mylink->GetLimit_Rz()->Set_active(true);
    mylink->GetLimit_Rz()->Set_min(-CH_C_PI_2);
    mylink->GetLimit_Rz()->Set_max(CH_C_PI_2);

    // now open the file for input
    /*
    std::stringstream ss("");
    ss << "mot" << i + 1 << ".dat";
    std::ifstream file(ss.str());
    if (!file.is_open())
    {
        exit(0);
    }
    std::vector<double> time;
    std::vector<double> data;
    ReadCSV(file, time, data);
    file.close();
    ChSharedPtr<ChFunction_Data> torqfunct(new ChFunction_Data(time, data));
    mylink->Set_tor_funct(torqfunct);
    ChSharedPtr<ChFunction_PID> torqfunct(
        new ChFunction_PID(10, 0, 0.4, mylink));
    torqfunct->dt = 1e-3;
    torqfunct->SetMax(1.1);
    torqfunct->SetMin(-1.1);
    mylink->Set_tor_funct(torqfunct);
    /*
    double ratio = 0.5;
    double period = 1 / ww;
    double offset = double(k) / kk;
    double value =  0.3 - 0.3 * cos(CH_C_2PI * double(k) / kk);
    if (int(k / 2) % 2 == 0)
    {
    value = -value;
    offset = offset + period * ratio;
    }
    ChSharedPtr<ChFunction_SquareWave> torqfunc(new ChFunction_SquareWave(ratio,
    period, value, offset));
    mylink->Set_tor_funct(torqfunc);
    */
}
