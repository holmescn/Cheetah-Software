#include "FoldLegsCtrl.h"

FoldLegsCtrl::FoldLegsCtrl(LegController<float> *legCtrl, StateEstimatorContainer<float> *stateEstimator)
: ActionCtrl(legCtrl, stateEstimator)
{
  // 定义角度 angle
  float a0 = 0.0f;
  float a1 = deg2rad(-80.0f);
  float a2 = deg2rad(145.0f);

  _target_jpos << a0, a1, a2,
                  a0, a1, a2,
                  a0, a1, a2,
                  a0, a1, a2;
}

int FoldLegsCtrl::step(Mat43f& init_jpos)
{
  _JointPD(init_jpos, _target_jpos, _motion_total_iterations);

  int ret = 0;
  if(++_iteration >= _motion_total_iterations) {
    _iteration = 0;
    init_jpos = _target_jpos;
    ret = 1;
  }

  return ret;
}
