#ifndef STATE_CTRL_H
#define STATE_CTRL_H
#include <RobotController.h>

typedef Eigen::Matrix<float, 3, 1> Vec3f;
typedef Eigen::Matrix<float, 4, 3> Mat43f;
typedef Eigen::Matrix<float, 3, 3> Mat33f;

class ActionCtrl {
protected:
  LegController<float> *_legCtrl;
  StateEstimatorContainer<float> *_stateEstimator;
  size_t _iteration;
  Mat33f _kp, _kd;

  void _UpdateJPos(Mat43f& jpos);
  Vec3f _interplation(int leg, const Mat43f &f, const Mat43f &t, float progress);
  void _JointPD(const Mat43f& init_jpos, const Mat43f& target_jpos, size_t motion_total_iterations);

  inline bool _IsUpsideDown() const {
    auto est = _stateEstimator->getResult();
    return est.rBody(2, 2) < 0;
  }

public:
  ActionCtrl(LegController<float> *legCtrl, StateEstimatorContainer<float> *stateEstimator);
  ActionCtrl(const ActionCtrl&) = delete;
  ActionCtrl(ActionCtrl&&) = delete;
  ActionCtrl& operator=(const ActionCtrl&) = delete;
  ActionCtrl& operator=(ActionCtrl&&) = delete;
  virtual ~ActionCtrl() = default;

public:
  virtual void init();
  virtual int step(Mat43f& init_jpos) = 0;

  static constexpr size_t LEG_F_R = 0;
  static constexpr size_t LEG_F_L = 1;
  static constexpr size_t LEG_B_R = 2;
  static constexpr size_t LEG_B_L = 3;

  static const Vec3f Vec3fZero;
};

inline Mat33f diag3f(float v) {
  Mat33f mat;
  mat << v, 0, 0,  0, v, 0,  0, 0, v;
  return mat;
}

inline Mat33f diag3f(float v1, float v2, float v3) {
  Mat33f mat;
  mat << v1, 0, 0,  0, v2, 0,  0, 0, v3;
  return mat;
}

#endif // STATE_CTRL_H