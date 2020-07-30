#include "ActionCtrl.h"

const Vec3f ActionCtrl::Vec3fZero = Vec3<float>::Zero();

ActionCtrl::ActionCtrl(LegController<float> *legCtrl, StateEstimatorContainer<float> *stateEstimator)
: _legCtrl(legCtrl),
  _stateEstimator(stateEstimator),
  _iteration(0)
{
  _kp = diag3f(20.0f);
  _kd = diag3f( 1.0f);
}

void ActionCtrl::init()
{
    _iteration = 0;
}

void ActionCtrl::_JointPD(const Mat43f& init_jpos, const Mat43f& target_jpos, size_t motion_total_iterations)
{
  float progress = _iteration * 1.0 / motion_total_iterations;
  for (size_t leg(0); leg<4; ++leg) {
    auto qDes = _interplation(leg, init_jpos, target_jpos, progress);
    _legCtrl->commands[leg].qDes = qDes;
    _legCtrl->commands[leg].qdDes = Vec3fZero;
    _legCtrl->commands[leg].kpJoint = _kp;
    _legCtrl->commands[leg].kdJoint = _kd;
  }
}

Vec3<float> ActionCtrl::_interplation(int leg, const Mat43f &f, const Mat43f &t, float progress)
{
  float a(0.0), b(1.0);
  if (progress <= 1.0) {
    b = progress;
    a = 1.0 - b;
  }

  Vec3f v;
  for(size_t i(0); i<3; ++i) {
    v[i] = a*f(leg, i) + b*t(leg, i);
  }
  return v;
}

void ActionCtrl::_UpdateJPos(Mat43f& jpos)
{
  for(size_t leg(0); leg<4; ++leg) {
    for(size_t joint(0); joint<3; ++joint) {
      jpos(leg, joint) = _legCtrl->datas[leg].q[joint];
    }
  }
}
