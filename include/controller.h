#ifndef INCLUDE_CONTROLLER_H_
#define INCLUDE_CONTROLLER_H_

#include <vector>

#include <motion_functions/ChFunction_Base.h>
#include <physics/ChSystem.h>
#include <physics/ChLinkEngine.h>
#include <physics/ChContactContainerBase.h>

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

private:
  chrono::ChSystem *ch_system_;
  class Robot *robot_;
  // Contact force on each of the robot segment.
  chrono::ChReportContactCallback2 *contact_reporter_;
  std::vector<chrono::ChVector<> > contact_force_list_;
  // the torques at joints, computed from contact forces.
  chrono::ChVectorDynamic<> torques_media_;
  chrono::ChVectorDynamic<> torques_contact_;
  // Parametr for the CPG
  double omega_ = 0.2 * chrono::CH_C_2PI;
  double num_waves_ = 2.0;
  double default_amplitude_ = 0.6;
  chrono::ChVectorDynamic<> amplitudes_;
  chrono::ChVectorDynamic<> average_contact_weight_;
  // reference counting
  size_t steps_ = 0;
};

void UseController(Controller *controller);
void UsePositionControl(Robot *robot);

namespace chrono {
class ChFunctionController : public ChFunction {
public:
  ChFunctionController(size_t index, Controller *controller)
      : index_(index), controller_(controller) {}
  ~ChFunctionController() {}
  ChFunction *new_Duplicate() {
    return new ChFunctionController(index_, controller_);
  }
  int Get_Type() { return 9527; }
  double Get_y(double curr_t);
  double Get_y_dx(double new_t) { return 0; }

  double p_gain = 0.70;
  double i_gain = 0.00;
  double d_gain = 1e-3;
  double torque_limit = 1.0;
  double angle_limit = 1.2;

protected:
  double GetMediaTorque(double t);
  double GetContactTorque(double t);
  // The low level PID controller in motor.
  double ComputeDriveTorque(double t);
  double ComputeLimitTorque(double t);
  double cum_error_ = 0;
  Controller *controller_;
  size_t index_;
};
} // namespace chrono

#endif // INCLUDE_CHFUNCTION_CONTROLLER_H_
