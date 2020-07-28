#ifndef FOLD_LAGS_CTRL_H
#define FOLD_LAGS_CTRL_H
#include "StateCtrl.h"

class FoldLegsCtrl : public StateCtrl {
  inline bool _IsUpsideDown() const {
    auto est = _stateEstimator->getResult();
    return est.rBody(2, 2) < 0;
  }

  const float _motion_total_iterations = 400.0f;
  Mat34<float> _final_jpos;

public:
  FoldLegsCtrl(LegController<float> *legCtrl, StateEstimatorContainer<float> *stateEstimator);
  FoldLegsCtrl(const FoldLegsCtrl&) = delete;
  FoldLegsCtrl(FoldLegsCtrl&&) = delete;
  FoldLegsCtrl& operator=(const FoldLegsCtrl&) = delete;
  FoldLegsCtrl& operator=(FoldLegsCtrl&&) = delete;
  ~FoldLegsCtrl() override = default;

  int step(Mat34<float>& jPos) override;
};

#endif //FOLD_LAGS_CTRL_H