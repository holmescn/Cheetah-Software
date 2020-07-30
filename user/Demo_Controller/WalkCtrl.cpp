#include "WalkCtrl.h"

WalkCtrl::WalkCtrl(LegController<float> *legCtrl, StateEstimatorContainer<float> *stateEstimator)
: ActionCtrl(legCtrl, stateEstimator), _state(State::Step1)
{
  _step_iterations = 250;

  // 站定
  {
    float f1 = deg2rad(-45.0f);
    float f2 = deg2rad( 90.0f);
    float b1 = deg2rad(-45.0f);
    float b2 = deg2rad( 90.0f);
    _step0_jpos << 0.0, f1, f2,
                   0.0, f1, f2,
                   0.0, b1, b2,
                   0.0, b1, b2;
  }

  _step1_jpos = _step0_jpos;
  _step1_jpos(LEG_F_R, 1) = deg2rad(-60.0f);
  _step1_jpos(LEG_F_R, 2) = deg2rad(120.0f);
  _step1_jpos(LEG_F_L, 1) = deg2rad(-60.0f);
  _step1_jpos(LEG_F_L, 2) = deg2rad(120.0f);

  _step2_jpos = _step0_jpos;
  _step2_jpos(LEG_F_R, 1) = deg2rad(-10.0f);
  _step2_jpos(LEG_F_R, 2) = deg2rad( 30.0f);
  _step2_jpos(LEG_F_L, 1) = deg2rad(-10.0f);
  _step2_jpos(LEG_F_L, 2) = deg2rad( 30.0f);

  _step3_jpos = _step0_jpos;
  _step3_jpos(LEG_B_R, 1) = deg2rad(-45.0f);
  _step3_jpos(LEG_B_R, 2) = deg2rad( 30.0f);
  _step3_jpos(LEG_B_L, 1) = deg2rad(-45.0f);
  _step3_jpos(LEG_B_L, 2) = deg2rad( 30.0f);

  // 伸出左前和右后腿
  _step4_jpos = _step0_jpos;
}

void WalkCtrl::_NextState(State nextState, size_t step_iterations, Mat43f& jpos)
{
  _iteration = 0;
  _step_iterations = step_iterations;
  _UpdateJPos(jpos);
  _state = nextState;
}

int WalkCtrl::step(Mat43f& init_jpos)
{
  float progress = ++_iteration * 1.0 / _step_iterations;
  switch (_state) {
  case State::Step1:
    if (progress <= 1.0) {
      _kp = diag3f(100.0f);
      _kd = diag3f(1.0f);
      _JointPD(init_jpos, _step1_jpos, _step_iterations);
    } else {
      _NextState(State::Step2, 20, init_jpos);
    }
    break;
  case State::Step2:
    if (progress <= 1.0) {
      for (size_t leg(0); leg<4; ++leg) {
        _legCtrl->commands[leg].qDes = Vec3fZero;
        _legCtrl->commands[leg].qdDes = Vec3fZero;
      }
      auto tau = Vec3f(0.0f, 25.0f, -22.0f);
      _legCtrl->commands[LEG_F_L].tauFeedForward = tau;
      _legCtrl->commands[LEG_F_R].tauFeedForward = tau;
    } else {
      _NextState(State::Step3, 40, init_jpos);
    }
    break;
  case State::Step3:
    if (progress && progress <= 1.0) {
      if (_iteration > 10) {
        for (size_t leg(0); leg<4; ++leg) {
          _legCtrl->commands[leg].qDes = Vec3fZero;
          _legCtrl->commands[leg].qdDes = Vec3fZero;
        }
        auto tau = Vec3<float>(0.0f, -10.0f, -30.0f);
        _legCtrl->commands[LEG_B_L].tauFeedForward = tau;
        _legCtrl->commands[LEG_B_R].tauFeedForward = tau;
        tau = Vec3<float>(0.0f, -5.0f, 5.0f);
        _legCtrl->commands[LEG_F_L].tauFeedForward = tau;
        _legCtrl->commands[LEG_F_R].tauFeedForward = tau;
      }
    } else {
      _NextState(State::Step4, 40, init_jpos);
    }
    break;
  case State::Step4:
    if (progress <= 1.0) {
      _kp = diag3f(100.0f);
      _kd = diag3f(10.0f);
      _JointPD(init_jpos, _step4_jpos, _step_iterations);
    } else {
      _NextState(State::Step1, 250, init_jpos);
    }
    break;
  case State::End:
    return 1;
  default:
    puts("Unknown state");
    break;
  }

  return 0;
}
