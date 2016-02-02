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

struct WaveWindow {
  // The starting joint index of the window, allowing negative value.
  int window_start;
  // The span of the window, which is always assumed to be a half wave length.
  int window_width;
  // The window wave parameters
  double amplitude;
  double amp_modifier = 1.0;
  // The temporal frequency is not a independent parameter. Once the wave group
  // speed is determined the temporal frequency is extracted from the width of
  // the window.
  double frequency;
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
  // Process the commands in the queue, one at a time
  void ProcessCommandQueue(double dt);

  // Generate a new window, maybe taken from the recycled window
  WaveWindow GenerateDefaultWindow();
  // Initialize a window coverage
  void InitializeWindows();
  // Propagate window
  void PropagateWindows(double dt);
  // Update window params using the compliance params
  void UpdateWindowParams(double dt);
  // Apply window params to motor functions
  void ApplyWindowParams();

  // The torques at each joint contributed from the contact force and/or media
  // resistance.
  Eigen::VectorXd ComputeInternalTorque();

  // Core components
  chrono::ChSystem *ch_system_;
  class Robot *robot_;

  // Contact force on each of the robot segment.
  chrono::ContactExtractor *contact_reporter_;

  // Parameters for the position control.
  double num_waves_ = 1.0;
  double default_frequency_ = 0.10 * chrono::CH_C_2PI;
  double default_amplitude_ = 0.35;
  double group_velocity_ = default_frequency_ / 2.0 / chrono::CH_C_2PI;
  // Parameters for advanced control.
  double command_frequency_ = default_frequency_;
  double command_amplitude_ = default_amplitude_;

  // Buffers for wave windows.
  std::list<WaveWindow> wave_windows_;
  // Recycled wave windows
  std::list<WaveWindow> past_windows_;
  // motor functions to be adjusted based on the window it belongs to
  std::vector<chrono::ChSharedPtr<chrono::ChFunctionMotor>> motor_functions_;

  // TODO
  std::queue<Json::Value> command_queue_;
  // TODO
  int command_count_down_ = 0;
  // step counting
  size_t steps_ = 0;
  // current command counting; TODO
  size_t command_count_ = 0;
};
#endif // INCLUDE_CHFUNCTION_CONTROLLER_H_
