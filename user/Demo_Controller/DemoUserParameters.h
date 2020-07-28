#ifndef PROJECT_JPOSUSERPARAMETERS_H
#define PROJECT_JPOSUSERPARAMETERS_H

#include "ControlParameters/ControlParameters.h"

class DemoUserParameters : public ControlParameters {
public:
  DemoUserParameters()
      : ControlParameters("user-parameters"),
        INIT_PARAMETER(tau_ff),
        INIT_PARAMETER(kp),
        INIT_PARAMETER(kd),
        INIT_PARAMETER(joint0),
        INIT_PARAMETER(joint1),
        INIT_PARAMETER(joint2),
        INIT_PARAMETER(leg)
      {}

  DECLARE_PARAMETER(double, tau_ff);
  DECLARE_PARAMETER(double, kp);
  DECLARE_PARAMETER(double, kd);
  DECLARE_PARAMETER(double, joint0);
  DECLARE_PARAMETER(double, joint1);
  DECLARE_PARAMETER(double, joint2);
  DECLARE_PARAMETER(double, leg);
};

#endif //PROJECT_JPOSUSERPARAMETERS_H
