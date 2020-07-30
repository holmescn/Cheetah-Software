#include "Demo_Controller.hpp"
#include "StandUpCtrl.h"
#include "FoldLegsCtrl.h"
#include "WalkCtrl.h"


void DemoController::initializeController() {
  _jpos.setZero();
}

/**
 * leg(0) 右前腿
 * joint(0) 脚 joint(1) 肩 joint(2) 肘
 */
void DemoController::runController() {
  switch (_state) {
    case State::Init:
    _UpdateJPos();
    if (++_iteration >= 10) _state = State::StandUp;
    break;
  case State::StandUp:
    _DoActionForState<StandUpCtrl>(State::Walk);
    break;
  case State::Walk:
    _DoActionForState<WalkCtrl>(State::FoldLegs);
    break;
  case State::FoldLegs:
    _DoActionForState<FoldLegsCtrl>(State::StandUp);
    break;
  case State::Idle:
    break;
  default:
    assert(false);
    break;
  }
}

void DemoController::_UpdateJPos() {
  for(size_t leg(0); leg<4; ++leg) {
    for(size_t joint(0); joint<3; ++joint) {
      _jpos(leg, joint) = _legController->datas[leg].q[joint];
    }
  }
}

void DemoController::debugPrint() {
  printf("%10s %10s %10s %10s %10s %10s\n", "leg/joint", "q", "qd", "p", "v", "tau");
  
  for (int leg(0); leg < 4; ++leg) {
    for (int joint(0); joint < 3; ++joint) {
      double q = _legController->datas[leg].q[joint];
      double qd = _legController->datas[leg].qd[joint];
      double p = _legController->datas[leg].p[joint];
      double v = _legController->datas[leg].v[joint];
      double tau = _legController->datas[leg].tauEstimate[joint];
      q = rad2deg(q);
      qd = rad2deg(qd);
      printf("%6d/%d   %10.4f %10.4f %10.4f %10.4f %10.4f\n", leg, joint, q, qd, p, v, tau);
    }
     printf("\n");
  }

  printf("\n");
}