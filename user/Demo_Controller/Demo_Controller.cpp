#include "Demo_Controller.hpp"
#include "StandUpCtrl.h"
#include "FoldLegsCtrl.h"

void DemoController::initializeController() {
  _jpos.setZero();
  _standUpCtrl = new StandUpCtrl(_legController, _stateEstimator);
  _foldLegsCtrl = new FoldLegsCtrl(_legController, _stateEstimator);
}

/**
 * leg(0) 右前腿
 * joint(0) 脚 joint(1) 肩 joint(2) 肘
 */
void DemoController::runController() {
  if (++_iteration < 10) {
    _UpdateJPos();
  } else {
    switch (_state)
    {
    case State::PreStandUp:
      _UpdateJPos();
      _standUpCtrl->init();
      _state = State::StandUp;
      break;
    case State::StandUp:
      if (_standUpCtrl->step(_jpos)) {
        _state = State::PreFoldLegs;
      }
      break;
    case State::PreFoldLegs:
      _UpdateJPos();
      _foldLegsCtrl->init();
      _state = State::FoldLegs;
      break;
    case State::FoldLegs:
      if (_foldLegsCtrl->step(_jpos)) {
        _state = State::PreStandUp;
      }
      break;
    default:
      assert(false);
      break;
    }
  }
}

void DemoController::_StandUp() {
  if (_standUpCtrl->step(_jpos)) {

  }
}

void DemoController::_UpdateJPos() {
  for(size_t leg(0); leg<4; ++leg) {
    for(size_t joint(0); joint<3; ++joint) {
      _jpos(joint, leg) = _legController->datas[leg].q[joint];
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