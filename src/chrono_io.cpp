#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <physics/ChSystem.h>

#include "include/chrono_io.h"
#include "include/contact_reporter.h"
#include "include/vector_utility.h"

using namespace chrono;

void LogBodies(std::vector<chrono::ChSharedPtr<chrono::ChBody>> &body_list,
               std::ostream &stream) {
  stream << std::setprecision(8);
  stream << std::scientific;
  for (size_t i = 0; i < body_list.size(); ++i) {
    auto cur_body = body_list[i];
    stream << cur_body->GetIdentifier() << " ";
    stream << cur_body->GetPos() << " " << cur_body->GetPos_dt() << " ";
    auto rot_quat = cur_body->GetRot();
    // needs to conjugate to satisfy the matlab convention
    rot_quat.Conjugate();
    stream << rot_quat << " ";
    // now output the angular velocity
    stream << cur_body->GetWvel_par() << "\n";
  }
  stream.flush();
}

void LogMotors(
    std::vector<chrono::ChSharedPtr<chrono::ChServoMotor>> &motor_list,
    std::ostream &stream) {
  stream << std::setprecision(8);
  stream << std::scientific;
  for (size_t i = 0; i < motor_list.size(); ++i) {
    stream << motor_list[i]->GetMotorID() << " ";
    stream << motor_list[i]->GetMotorRotation() << " ";
    stream << motor_list[i]->GetMotorTorque() << "\n";
  }
  stream.flush();
}
