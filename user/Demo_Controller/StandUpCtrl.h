#ifndef STAND_UP_CTRL_H
#define STAND_UP_CTRL_H
#include "StateCtrl.h"

class StandUpCtrl : public StateCtrl {
  inline bool _IsUpsideDown() const {
    auto est = _stateEstimator->getResult();
    return est.rBody(2, 2) < 0;
  }

  const float _motion_total_iterations = 250.0f;
  Mat34<float> _final_jpos;

public:
    StandUpCtrl(LegController<float> *legCtrl, StateEstimatorContainer<float> *stateEstimator);
    StandUpCtrl(const StandUpCtrl&) = delete;
    StandUpCtrl(StandUpCtrl&&) = delete;
    StandUpCtrl& operator=(const StandUpCtrl&) = delete;
    StandUpCtrl& operator=(StandUpCtrl&&) = delete;
    ~StandUpCtrl() override = default;

    int step(Mat34<float>& jPos) override;
};

#endif // STAND_UP_CTRL_H
