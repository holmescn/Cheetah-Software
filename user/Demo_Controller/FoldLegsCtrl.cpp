#include "FoldLegsCtrl.h"

FoldLegsCtrl::FoldLegsCtrl(LegController<float> *legCtrl, StateEstimatorContainer<float> *stateEstimator)
: StateCtrl(legCtrl, stateEstimator)
{
  // 定义角度 angle
  float a0  = 0.0f;
  float a1 = deg2rad(-80.0f);
  float a2 = deg2rad(150.0f);

  _final_jpos << a0, a0, a0, a0,
                 a1, a1, a1, a1,
                 a2, a2, a2, a2;

  _kp = make_mat3f(10.0f);
}

int FoldLegsCtrl::step(Mat34<float>& jPos)
{
  float progress = _iteration / _motion_total_iterations;
  for(size_t leg(0); leg<4; ++leg) {
    Vec3<float> qDes = _interplation(leg, jPos, _final_jpos, progress);
    _SetJointPD(leg, qDes);
  }

  int ret = 0;
  if(++_iteration >= _motion_total_iterations) {
    jPos = _final_jpos;
    _iteration = 0;
    ret = 1;
  }

  return ret;
}
