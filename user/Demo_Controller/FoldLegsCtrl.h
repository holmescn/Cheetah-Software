#ifndef FOLD_LAGS_CTRL_H
#define FOLD_LAGS_CTRL_H
#include "ActionCtrl.h"

class FoldLegsCtrl : public ActionCtrl {
  const size_t _motion_total_iterations = 400.0f;
  Mat43f _target_jpos;

public:
  FoldLegsCtrl(LegController<float> *legCtrl, StateEstimatorContainer<float> *stateEstimator);
  FoldLegsCtrl(const FoldLegsCtrl&) = delete;
  FoldLegsCtrl(FoldLegsCtrl&&) = delete;
  FoldLegsCtrl& operator=(const FoldLegsCtrl&) = delete;
  FoldLegsCtrl& operator=(FoldLegsCtrl&&) = delete;
  ~FoldLegsCtrl() override = default;

  int step(Mat43f& init_jpos) override;
};

#endif //FOLD_LAGS_CTRL_H