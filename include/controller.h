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
  double wave_speed = 0.05;
  double desired_amplitude = 0.35;
  // the amplitude modifier
  std::vector<double> amplitudes;
  std::vector<double> amplitudes_dt;
  // The temporal frequency and number of waves are dependent. Once the wave
  // group speed is determined the number of waves are extracted.
  // Suppose we use 1.5 waves along the body
  double frequency = 2 * chrono::CH_C_2PI * wave_speed * 1.5;
  std::vector<double> phases;
  std::vector<double> phases_dt;
  std::vector<double> desired_phases;

  std::vector<double> theta;
  std::vector<double> theta_dt;
};

class Controller {
public:
  Controller(chrono::ChSystem *ch_system, class Robot *i_robot);
  // Step the controller
  void Step(double dt);
  // get the toruqe for joint i
  size_t GetNumMotors();

  // The PID based positio control
  void EnablePIDMotorControl();
  // The "perfect" control
  void EnablePosMotorControl();
  // Change the undultation amplitude of the snake
  void SetDefaultParams(const Json::Value &command);

private:
  // Generate a new window, maybe taken from the recycled window
  void InitializeWaveParams();
  // Update wave params using the compliance params
  void UpdateAmplitudes(double dt);
  void UpdatePhases(double dt);
  void UpdateAngles(double dt);
  // Apply window params to motor functions
  void PropagateWaveParams(double dt);
  void ApplyWaveParams();

  void ExtractContactForces();
  void ApplyHeadStrategy();
  int head_strategy_count_down_ = -100;
  int head_index_ = 0;

  // Grab and glide control based on torque. First we determine whether to grab.
  std::vector<size_t> CharacterizeContacts();

  // The torques at each joint contributed from the contact force and/or media
  // resistance.
  Eigen::VectorXd ComputeInternalTorque();

  // Core components
  chrono::ChSystem *ch_system_;
  class Robot *robot_;

  // Contact force on each of the robot segment.
  chrono::ContactExtractor *contact_reporter_;
  std::vector<chrono::ChVector<>> contact_forces_;

  WaveParams wave_params_;
  // Parameters for the position control.
  // double default_amplitude_ = 0.40;
  // double default_frequency_ = 0.20 * chrono::CH_C_2PI;
  // double group_velocity_ = 0.05;
  // double num_waves_ = default_frequency_ / chrono::CH_C_2PI /
  // group_velocity_;

  // motor functions to be adjusted based on the window it belongs to
  std::vector<std::shared_ptr<chrono::ChFunctionMotor>> motor_functions_;
  // step count
  size_t steps_ = 0;
};
#endif // INCLUDE_CHFUNCTION_CONTROLLER_H_
