#include "StandUpCtrl.h"

StandUpCtrl::StandUpCtrl(LegController<float> *legCtrl, StateEstimatorContainer<float> *stateEstimator)
: ActionCtrl(legCtrl, stateEstimator)
{
  // 定义站立时的角度
  float f0 = deg2rad(  0.0f);
  float f1 = deg2rad(-45.0f);
  float f2 = deg2rad( 90.0f);
  float b0 = deg2rad(  0.0f);
  float b1 = deg2rad(-55.0f);
  float b2 = deg2rad( 80.0f);

  _target_jpos << f0, f1, f2,
                  f0, f1, f2,
                  b0, b1, b2,
                  b0, b1, b2;
}

int StandUpCtrl::step(Mat43f& init_jpos)
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
    _JointPD(init_jpos, _target_jpos, _motion_total_iterations);

    auto kp = diag3f(40.0f);
    _legCtrl->commands[LEG_B_L].kpJoint = kp;
    _legCtrl->commands[LEG_B_R].kpJoint = kp;
  }

  // feed forward mass of robot.
  // for(int i = 0; i < 4; i++) {
  //   _legController->commands[i].forceFeedForward = f_ff;
  // }
  Vec4<float> se_contactState(0.5, 0.5, 0.5, 0.5);
  _stateEstimator->setContactPhase(se_contactState);
  return _iteration >= _motion_total_iterations;
}
