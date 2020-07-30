#ifndef STAND_UP_CTRL_H
#define STAND_UP_CTRL_H
#include "ActionCtrl.h"

class StandUpCtrl : public ActionCtrl {
  const size_t _motion_total_iterations = 250.0f;
  Mat43f _target_jpos;

public:
    StandUpCtrl(LegController<float> *legCtrl, StateEstimatorContainer<float> *stateEstimator);
    StandUpCtrl(const StandUpCtrl&) = delete;
    StandUpCtrl(StandUpCtrl&&) = delete;
    StandUpCtrl& operator=(const StandUpCtrl&) = delete;
    StandUpCtrl& operator=(StandUpCtrl&&) = delete;
    ~StandUpCtrl() override = default;

    int step(Mat43f& init_jpos) override;
};

#endif // STAND_UP_CTRL_H
