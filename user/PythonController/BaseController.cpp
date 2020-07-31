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
        _fn_initialize = o.attr("initialize");
    }

    _fn_run = py::cast<py::none>(Py_None);
    if (py::hasattr(o, "run")) {
        _fn_run = o.attr("run");
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

ControlParameters* BaseController::getUserControlParameters()
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

std::unique_ptr<LegProperty> BaseController::GetLeg() const
{
    return std::make_unique<LegProperty>(_legController);
}

std::unique_ptr<StateProperty> BaseController::GetState() const
{
    return std::make_unique<StateProperty>(_stateEstimator);
}

Matrix43f BaseController::GetJointMatrix(Eigen::Vector3f LegControllerData<float>::* member) const
{
    Matrix43f m;
    for (int l(0); l<4; ++l) {
        for (int j(0); j<3; ++j) {
            m(l, j) = (_legController->datas[l].*member)[j];
        }
    }
    return m;
}

void BaseController::SetJointMatrix(Eigen::Vector3f LegControllerCommand<float>::* member, const Matrix43f &m)
{
    for (int l(0); l<4; ++l) {
        for (int j(0); j<3; ++j) {
            (_legController->commands[l].*member)[j] = m(l, j);
        }
    }
}

void BaseController::SetJointMatrix(Eigen::Matrix3f LegControllerCommand<float>::* member, const Matrix43f &m)
{
    for (int l(0); l<4; ++l) {
        for (int j(0); j<3; ++j) {
            (_legController->commands[l].*member)(j, j) = m(l, j);
        }
    }
}

Matrix43f BaseController::GetJointAngular() const
{
    if (_legController) {
        return GetJointMatrix(&LegControllerData<float>::q);
    }
    return Matrix43f::Zero();
}

Matrix43f BaseController::GetJointAngularVelocity() const
{
    if (_legController) {
        return GetJointMatrix(&LegControllerData<float>::qd);
    }
    return Matrix43f::Zero();
}

Matrix43f BaseController::GetJointPosition() const
{
    if (_legController) {
        return GetJointMatrix(&LegControllerData<float>::p);
    }
    return Matrix43f::Zero();
}

Matrix43f BaseController::GetJointVelocity() const
{
    if (_legController) {
        return GetJointMatrix(&LegControllerData<float>::v);
    }
    return Matrix43f::Zero();
}

void BaseController::SetJointAngular(Matrix43f m)
{
    if (_legController) {
        SetJointMatrix(&LegControllerCommand<float>::qDes, m);
    }
}

void BaseController::SetJointAngularVelocity(Matrix43f m)
{
    if (_legController) {
        SetJointMatrix(&LegControllerCommand<float>::qdDes, m);
    }
}

void BaseController::SetJointPosition(Matrix43f m)
{
    if (_legController) {
        SetJointMatrix(&LegControllerCommand<float>::pDes, m);
    }
}

void BaseController::SetJointVelocity(Matrix43f m)
{
    if (_legController) {
        SetJointMatrix(&LegControllerCommand<float>::vDes, m);
    }
}

void BaseController::SetJointTau(Matrix43f m)
{
    if (_legController) {
        SetJointMatrix(&LegControllerCommand<float>::tauFeedForward, m);
    }
}

void BaseController::SetJointForce(Matrix43f m)
{
    if (_legController) {
        SetJointMatrix(&LegControllerCommand<float>::forceFeedForward, m);
    }
}

void BaseController::SetKpJoint(Matrix43f m)
{
    if (_legController) {
        SetJointMatrix(&LegControllerCommand<float>::kpJoint, m);
    }
}

void BaseController::SetKdJoint(Matrix43f m)
{
    if (_legController) {
        SetJointMatrix(&LegControllerCommand<float>::kdJoint, m);
    }
}

void BaseController::SetKpCartesian(Matrix43f m)
{
    if (_legController) {
        SetJointMatrix(&LegControllerCommand<float>::kpCartesian, m);
    }
}

void BaseController::SetKdCartesian(Matrix43f m)
{
    if (_legController) {
        SetJointMatrix(&LegControllerCommand<float>::kdCartesian, m);
    }
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

void PyBaseController::initializeController()
{
    PYBIND11_OVERLOAD(
        void,            /* Return type */
        BaseController,  /* Parent class */
        initializeController,
    );
}

void PyBaseController::runController()
{
    PYBIND11_OVERLOAD(
        void,            /* Return type */
        BaseController,  /* Parent class */
        runController,
    );
}

void PyBaseController::updateVisualization()
{
    PYBIND11_OVERLOAD(
        void,            /* Return type */
        BaseController,  /* Parent class */
        updateVisualization,
    );
}

ControlParameters* PyBaseController::getUserControlParameters()
{
    PYBIND11_OVERLOAD(
        ControlParameters*,
        BaseController,
        getUserControlParameters,
    );
}
