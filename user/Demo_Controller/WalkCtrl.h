#ifndef WALK_CTRL_H
#define WALK_CTRL_H
#include "ActionCtrl.h"

class WalkCtrl : public ActionCtrl {
  size_t _step_iterations;

  Mat43f _step0_jpos;
  Mat43f _step1_jpos;
  Mat43f _step2_jpos;
  Mat43f _step3_jpos;
  Mat43f _step4_jpos;
  enum class State {
    Step1,
    Step2,
    Step3,
    Step4,
    End
  } _state;

  void _NextState(State nextState, size_t step_iterations, Mat43f& jpos);

public:
  WalkCtrl(LegController<float> *legCtrl, StateEstimatorContainer<float> *stateEstimator);
  WalkCtrl(const WalkCtrl&) = delete;
  WalkCtrl(WalkCtrl&&) = delete;
  WalkCtrl& operator=(const WalkCtrl&) = delete;
  WalkCtrl& operator=(WalkCtrl&&) = delete;
  ~WalkCtrl() override = default;

  int step(Mat43f& init_jpos) override;
};

#endif // WALK_CTRL_H
