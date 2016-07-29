#ifndef INCLUDE_CONTROLLER_H_
#define INCLUDE_CONTROLLER_H_

#include <list>
#include <queue>
#include <vector>

#include <Eigen/Core>
#include <json/json.h>
#include <physics/ChLinkEngine.h>
#include <physics/ChSystem.h>

#include "include/motor_function.h"

namespace chrono {
class ContactExtractor;
}

struct WaveParams {
  // The normalized wave_speed;
  double wave_speed = 0.1;
  double amplitude = 0.35;
  double head_phase = 0.0;

  // The temporal frequency and number of waves are dependent. Once the wave
  // group speed is determined the number of waves are extracted.
  // Suppose we use 1.5 waves along the body
  double frequency = 2 * chrono::CH_C_2PI * wave_speed * 1.5;
  std::vector<double> theta;
  std::vector<double> theta_dt;
  std::vector<double> theta_ref;
};

class Controller {
public:
  Controller(chrono::ChSystem *ch_system, class Robot *i_robot);
  // Step the controller
  void Step(double dt);
  size_t GetNumMotors();
  // Get the phase of i_th motor;
  double GetPhase(size_t i);
  // The PID based positio control
  void EnablePIDMotorControl();
  // The "perfect" control
  void EnablePosMotorControl();
  // Change the undultation amplitude of the snake
  void SetDefaultParams(const Json::Value &command);

private:
  // Generate a new window, maybe taken from the recycled window
  void InitializeWaveParams();
  void PropagateWave();
  void UpdateSnakeShape();
  void ExtractContactForces();
  void Wrap();
  void Wiggle();
  int num_contacts_ = 0;
  int contact_index_ = 0;
  // Grab and glide control based on torque. First we determine whether to grab.
  std::vector<size_t> CharacterizeContacts();

  // Core components
  chrono::ChSystem *ch_system_;
  class Robot *robot_;

  // Contact force on each of the robot segment.
  chrono::ContactExtractor *contact_reporter_;
  std::vector<chrono::ChVector<>> contact_forces_;

  WaveParams wave_params_;
  // motor functions to be adjusted based on the window it belongs to
  std::vector<std::shared_ptr<chrono::ChFunctionMotor>> motor_functions_;
  // step count
  size_t steps_ = 0;
};
#endif // INCLUDE_CHFUNCTION_CONTROLLER_H_
