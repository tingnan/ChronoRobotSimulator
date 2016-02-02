#ifndef INCLUDE_MOTOR_FUNCTION_H_
#define INCLUDE_MOTOR_FUNCTION_H_

#include <motion_functions/ChFunction_Base.h>

namespace chrono {

// A nonlinear phased based sinusoidal function
class ChFunctionMotor : public ChFunction {
public:
  ChFunctionMotor(double amp, double freq, double initial_phase)
      : amplitude_(amp), frequency_(freq), cumulated_phase_(initial_phase) {}
  ~ChFunctionMotor() {}
  ChFunction *new_Duplicate() {
    return new ChFunctionMotor(amplitude_, frequency_, cumulated_phase_);
  }
  int Get_Type() { return 8607; }
  void Step(double dt) { cumulated_phase_ += frequency_ * dt; }
  double Get_y(double t) { return amplitude_ * sin(cumulated_phase_); }
  double Get_y_dx(double t) {
    return amplitude_ * frequency_ * cos(cumulated_phase_);
  }
  void SetAmplitude(double amp) { amplitude_ = amp; }
  void SetFrequency(double freq) { frequency_ = freq; }
  double GetAmplitude() { return amplitude_; }
  double GetFrequency() { return frequency_; }

protected:
  double amplitude_ = 0;
  double frequency_ = 0;
  double cumulated_phase_ = 0;
};

} // namespace chrono

#endif // INCLUDE_MOTOR_FUNCTION_H_
