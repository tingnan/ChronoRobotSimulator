#ifndef INCLUDE_CHRONOIO_H_
#define INCLUDE_CHRONOIO_H_

#include <fstream>
#include <string>
#include <vector>

#include "include/rft.h"
#include "include/servo_motor.h"

void LogBodies(std::vector<chrono::ChSharedPtr<chrono::ChBody>> &body_list,
               std::ostream &stream);

void LogMotors(
    std::vector<chrono::ChSharedPtr<chrono::ChServoMotor>> &motor_list,
    std::ostream &stream);

#endif // INCLUDE_CHRONOIO_H_
