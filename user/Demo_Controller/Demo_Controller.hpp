#ifndef DEMO_CONTROLLER
#define DEMO_CONTROLLER

#include <RobotController.h>
#include "DemoUserParameters.h"
#include "StateCtrl.h"

class DemoController : public RobotController {
  enum class State {
    PreStandUp,
    StandUp,
    PreFoldLegs,
    FoldLegs,
  } _state;

public:
  DemoController() : RobotController(), _iteration(0) {
    
  }
  ~DemoController() override {
    delete _standUpCtrl;
  }

  virtual void initializeController();
  virtual void runController();
  virtual void updateVisualization(){}
  virtual ControlParameters* getUserControlParameters() {
    return &userParameters;
  }

private:
  void debugPrint();
  void _StandUp();
  void _UpdateJPos();

protected:
  size_t _iteration;
  StateCtrl *_standUpCtrl;
  StateCtrl *_foldLegsCtrl;
  Mat34<float> _jpos;
  DemoUserParameters userParameters;
};

#endif // DEMO_CONTROLLER
