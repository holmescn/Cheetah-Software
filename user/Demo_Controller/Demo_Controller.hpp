#ifndef DEMO_CONTROLLER
#define DEMO_CONTROLLER

#include <RobotController.h>
#include "DemoUserParameters.h"
#include "ActionCtrl.h"

class DemoController : public RobotController {
  enum class State {
    Init,
    StandUp,
    Walk,
    FoldLegs,
    Idle,
  } _state;

public:
  DemoController() : _state(State::Init), _iteration(0), _actionCtrl(nullptr) {
  }
  ~DemoController() override {
    if (_actionCtrl) {
      delete _actionCtrl;
    }
  }

  virtual void initializeController();
  virtual void runController();
  virtual void updateVisualization(){}
  virtual ControlParameters* getUserControlParameters() {
    return &userParameters;
  }

private:
  void debugPrint();
  void _UpdateJPos();
  template<class T>
  void _DoActionForState(State nextState) {
    if (_actionCtrl == nullptr) {
      _actionCtrl = new T(_legController, _stateEstimator);
      _actionCtrl->init();
    } else if (_actionCtrl->step(_jpos)) {
      delete _actionCtrl;
      _actionCtrl = nullptr;
      _state = nextState;
    }
  }

protected:
  size_t _iteration;
  ActionCtrl *_actionCtrl;
  Mat43f _jpos;
  DemoUserParameters userParameters;
};

#endif // DEMO_CONTROLLER
