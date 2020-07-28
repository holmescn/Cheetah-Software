#ifndef STATE_CTRL_H
#define STATE_CTRL_H
#include <RobotController.h>

class StateCtrl {
protected:
  LegController<float> *_legCtrl;
  StateEstimatorContainer<float> *_stateEstimator;
  size_t _iteration;
  Mat3<float> _kp, _kd;

  void _SetJointPD(int leg, const Vec3<float>& qDes, const Vec3<float>& qdDes=Vec3<float>::Zero());
  Vec3<float> _interplation(int leg, const Mat34<float> &f, const Mat34<float> &t, float progress);

public:
  StateCtrl(LegController<float> *legCtrl, StateEstimatorContainer<float> *stateEstimator);
  StateCtrl(const StateCtrl&) = delete;
  StateCtrl(StateCtrl&&) = delete;
  StateCtrl& operator=(const StateCtrl&) = delete;
  StateCtrl& operator=(StateCtrl&&) = delete;
  virtual ~StateCtrl() = default;

public:
  virtual void init();
  virtual int step(Mat34<float>& jPos) = 0;
};

inline Mat3<float> make_mat3f(float v) {
  Mat3<float> mat;
  mat << v, 0, 0,  0, v, 0,  0, 0, v;
  return mat;
}

inline Mat3<float> make_mat3f(float v1, float v2, float v3) {
  Mat3<float> mat;
  mat << v1, 0, 0,  0, v2, 0,  0, 0, v3;
  return mat;
}

#endif // STATE_CTRL_H