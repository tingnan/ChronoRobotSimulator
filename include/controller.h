#ifndef INCLUDE_CONTROLLER_H_
#define INCLUDE_CONTROLLER_H_

#include <vector>

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

private:
  chrono::ChSystem *ch_system_;
  class Robot *robot_;
  // Contact force on each of the robot segment.
  chrono::ChReportContactCallback2 *contact_reporter_;
  std::vector<chrono::ChVector<> > contact_force_list_;
  // the torques at joints, computed from contact forces.
  Eigen::VectorXd torques_media_;
  Eigen::VectorXd torques_contact_;
  // Parametr for the CPG
  double omega_ = 0.2 * chrono::CH_C_2PI;
  double num_waves_ = 2.0;
  double default_amplitude_ = 0.6;
  Eigen::VectorXd amplitudes_;
  Eigen::VectorXd average_contact_weight_;
  // reference counting
  size_t steps_ = 0;
};
#endif // INCLUDE_CHFUNCTION_CONTROLLER_H_
