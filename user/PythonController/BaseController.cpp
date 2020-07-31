#pragma GCC diagnostic ignored "-Wshadow"

#include <pybind11/pybind11.h>
#include "BaseController.h"
#include "PropertyHelper.h"

namespace py = pybind11;

BaseController::BaseController()
{
    //
}

void BaseController::SetPyInstance(pybind11::object o)
{
    _py_instance = o;
    _fn_initialize = py::cast<py::none>(Py_None);
    if (py::hasattr(o, "initialize")) {
        _fn_initialize = o["initialize"];
    }

    _fn_run = py::cast<py::none>(Py_None);
    if (py::hasattr(o, "run")) {
        _fn_run = o["run"];
    }
}

void BaseController::initializeController()
{
    if (!_fn_initialize.is_none()) {
        _fn_initialize();
    }
}

void BaseController::runController()
{
    if (!_fn_run.is_none()) {
        _fn_run();
    }
}

void BaseController::updateVisualization()
{
    // ignored right now.
}

ControlParameters* getUserControlParameters()
{
    return nullptr;
}

void BaseController::SetEnable(bool bEnable)
{
    _legController->_legsEnabled = bEnable;
}

void BaseController::SetMaxTorque(float maxTorque)
{
    _legController->_maxTorque = maxTorque;
}

void BaseController::SetEncodeZeros(bool bEnable)
{
    _legController->_zeroEncoders = bEnable;
}

void BaseController::SetCalibrate(uint32_t calibrate)
{
    _legController->_calibrateEncoders = calibrate;
}

std::unique_ptr<LegProxy> BaseController::GetLeg(size_t leg) const
{
    if (leg > 3) {
        throw std::invalid_argument("leg should be in [0, 3].");
    }
    return std::make_unique<LegProxy>(_legController, leg);
}

std::unique_ptr<StateProxy> BaseController::GetState() const
{
    return std::make_unique<StateProxy>(_stateEstimator);
}

Eigen::Matrix<float, 4, 3> BaseController::GetJointAngular() const
{
    Eigen::Matrix<float, 4, 3> m;
    for (int l(0); l<4; ++l) {
        for (int j(0); j<3; ++j) {
            m(l, j) = _legController->datas[l].q[j];
        }
    }
    return m;
}

Eigen::Matrix<float, 4, 3> BaseController::GetJointAngularVelocity() const
{
    Eigen::Matrix<float, 4, 3> m;
    for (int l(0); l<4; ++l) {
        for (int j(0); j<3; ++j) {
            m(l, j) = _legController->datas[l].qd[j];
        }
    }
    return m;
}

Eigen::Matrix<float, 4, 3> BaseController::GetJointPosition() const
{
    Eigen::Matrix<float, 4, 3> m;
    for (int l(0); l<4; ++l) {
        for (int j(0); j<3; ++j) {
            m(l, j) = _legController->datas[l].p[j];
        }
    }
    return m;
}

Eigen::Matrix<float, 4, 3> BaseController::GetJointVelocity() const
{
    Eigen::Matrix<float, 4, 3> m;
    for (int l(0); l<4; ++l) {
        for (int j(0); j<3; ++j) {
            m(l, j) = _legController->datas[l].v[j];
        }
    }
    return m;
}

void PyBaseController::initialize()
{
    PYBIND11_OVERLOAD_PURE(
        void,            /* Return type */
        BaseController,  /* Parent class */
        initialize,      /* Name of function in C++ (must match Python name) */
    );
}

void PyBaseController::run()
{
    PYBIND11_OVERLOAD_PURE(
        void,            /* Return type */
        BaseController,  /* Parent class */
        run,             /* Name of function in C++ (must match Python name) */
    );
}
