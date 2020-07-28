#include "StateCtrl.h"

StateCtrl::StateCtrl(LegController<float> *legCtrl, StateEstimatorContainer<float> *stateEstimator)
: _legCtrl(legCtrl),
  _stateEstimator(stateEstimator),
  _iteration(0)
{
  _kp = make_mat3f(20.0);
  _kd = make_mat3f(0.1);
}

void StateCtrl::init()
{
    _iteration = 0;
}

void StateCtrl::_SetJointPD(int leg, const Vec3<float>& qDes, const Vec3<float>& qdDes)
{  
  _legCtrl->commands[leg].qDes = qDes;
  _legCtrl->commands[leg].qdDes = qdDes;
  _legCtrl->commands[leg].kpJoint = _kp;
  _legCtrl->commands[leg].kdJoint = _kd;
}

Vec3<float> StateCtrl::_interplation(int leg, const Mat34<float> &f, const Mat34<float> &t, float progress)
{
  float a(0.0), b(1.0);
  if (0.0 <= progress && progress <= 1.0) {
    b = progress;
    a = 1.0 - b;
  }

  Vec3<float> v;
  for(size_t i(0); i<3; ++i) {
    v[i] = a*f(i, leg) + b * t(i, leg);
  }
  return v;
}
