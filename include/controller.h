#ifndef INCLUDE_CONTROLLER_H_
#define INCLUDE_CONTROLLER_H_

#include <queue>
#include <vector>

#include <json/json.h>
#include <Eigen/Core>
#include <motion_functions/ChFunction_Base.h>
#include <physics/ChSystem.h>
#include <physics/ChLinkEngine.h>

namespace chrono {
class ChReportContactCallback2;
}

class Controller {
public:
  Controller(chrono::ChSystem *ch_system, class Robot *i_robot);
  // Step the controller
  void Step(double dt);
  // get the toruqe for joint i
  size_t GetNumEngines();
  chrono::ChLinkEngine *GetEngine(size_t i);
  double GetMediaTorque(size_t i, double t);
  double GetContactTorque(size_t i, double t);
  double GetAngle(size_t i, double t);
  double GetAngularSpeed(size_t i, double t);

  void UseForceControl();
  void UsePositionControl();
  // Change the undultation amplitude of the snake
  void SetCommandAmplitude(double amp);
  void PushCommandToQueue(const Json::Value &command);

private:
  // Process the commands in the queue, one at a time
  void ProcessCommandQueue(double dt);
  chrono::ChSystem *ch_system_;
  class Robot *robot_;
  // Contact force on each of the robot segment.
  chrono::ChReportContactCallback2 *contact_reporter_;
  std::vector<chrono::ChVector<> > contact_force_list_;
  // the torques at joints, computed from contact forces.
  Eigen::VectorXd torques_media_;
  Eigen::VectorXd torques_contact_;
  // Parameters for the position control.
  double num_waves_ = 1.0;
  double default_frequency_ = 0.10 * chrono::CH_C_2PI;
  double default_amplitude_ = 0.30;
  // Parameters for advanced control.
  double command_frequency_ = default_frequency_;
  double command_amplitude_ = default_amplitude_;
  double group_velocity_ = default_frequency_ / 2.0 / chrono::CH_C_2PI;
  Eigen::VectorXd amplitudes_;
  Eigen::VectorXd frequencies_;
  Eigen::VectorXd cumulated_phases_;
  std::queue<Json::Value> command_queue_;
  int command_count_down_ = 0;
  // reference counting
  size_t steps_ = 0;
  // command_queue_counting;
  size_t command_count_ = 0;
};
#endif // INCLUDE_CHFUNCTION_CONTROLLER_H_
