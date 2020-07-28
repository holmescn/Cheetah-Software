#include "StandUpCtrl.h"

StandUpCtrl::StandUpCtrl(LegController<float> *legCtrl, StateEstimatorContainer<float> *stateEstimator)
: StateCtrl(legCtrl, stateEstimator)
{
  // 定义站立时的角度
  float a0  = 0.0f;
  float af1 = deg2rad(-45.0f);
  float af2 = deg2rad( 90.0f);
  float ab1 = deg2rad(-40.0f);
  float ab2 = deg2rad( 80.0f);

  _final_jpos << a0,  a0,  a0,  a0,
                 af1, af1, ab1, ab1,
                 af2, af2, ab2, ab2;

  _kp = make_mat3f(15.0);
}

int StandUpCtrl::step(Mat34<float>& jPos)
{
  auto est = _stateEstimator->getResult();
  float body_height = est.position[2];
  bool something_wrong = _IsUpsideDown() || body_height < 0.1;

  if( (++_iteration > floor(_motion_total_iterations*0.7) ) && something_wrong) {
    // If body height is too low because of some reason
    // even after the stand up motion is almost over
    // (Can happen when E-Stop is engaged in the middle of Other state)
    printf("[Warning] body height is still too low (%f) or UpsideDown (%d); Folding legs.\n",
      body_height, _IsUpsideDown());
  } else {
    float progress = _iteration / _motion_total_iterations;
    for(size_t leg(0); leg<4; ++leg){
      Vec3<float> qDes = _interplation(leg, jPos, _final_jpos, progress);
      _SetJointPD(leg, qDes);
    }
  }

  // feed forward mass of robot.
  // for(int i = 0; i < 4; i++) {
  //   _legController->commands[i].forceFeedForward = f_ff;
  // }
  Vec4<float> se_contactState(0.5, 0.5, 0.5, 0.5);
  _stateEstimator->setContactPhase(se_contactState);
  return _iteration >= _motion_total_iterations;
}
