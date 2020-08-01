#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wattributes"

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
    Matrix43f m = Matrix43f::Zero();
    if (_legController) {
        for (int l(0); l<4; ++l) {
            m.row(l) = _legController->datas[l].*member;
        }
    }
    return m;
}

void BaseController::SetJointMatrix(Eigen::Vector3f LegControllerCommand<float>::* member, const pybind11::object &o)
{
    if (_legController) {
        if (py::isinstance<py::float_>(o)) {
            Eigen::Vector3f v = Eigen::Vector3f::Ones() * o.cast<float>();
            for (int l(0); l<4; ++l) {
                _legController->commands[l].*member = v;
            }
        } else {
            auto m = o.cast<Matrix43f>();
            for (int l(0); l<4; ++l) {
                _legController->commands[l].*member = m.row(l);
            }
        }
    }
}

void BaseController::SetJointMatrix(Eigen::Matrix3f LegControllerCommand<float>::* member, const pybind11::object &o)
{
    if (_legController) {
        if (py::isinstance<py::float_>(o)) {
            float v = o.cast<float>();
            for (int l(0); l<4; ++l) {
                auto &mm = _legController->commands[l].*member;
                for (int j(0); j<3; ++j) {
                    mm(j, j) = v;
                }
            }
        } else {
            auto m = o.cast<Matrix43f>();
            for (int l(0); l<4; ++l) {
                auto &mm = _legController->commands[l].*member;
                for (int j(0); j<3; ++j) {
                    mm(j, j) = m(l, j);
                }
            }
        }
    }
}

Matrix43f BaseController::GetJointAngular() const
{
    return GetJointMatrix(&LegControllerData<float>::q);
}

Matrix43f BaseController::GetJointAngularVelocity() const
{
    return GetJointMatrix(&LegControllerData<float>::qd);
}

Matrix43f BaseController::GetJointPosition() const
{
    return GetJointMatrix(&LegControllerData<float>::p);
}

Matrix43f BaseController::GetJointVelocity() const
{
    return GetJointMatrix(&LegControllerData<float>::v);
}

void BaseController::SetJointAngular(pybind11::object o)
{
    SetJointMatrix(&LegControllerCommand<float>::qDes, o);
}

void BaseController::SetJointAngularVelocity(pybind11::object o)
{
    SetJointMatrix(&LegControllerCommand<float>::qdDes, o);
}

void BaseController::SetJointPosition(pybind11::object o)
{
    SetJointMatrix(&LegControllerCommand<float>::pDes, o);
}

void BaseController::SetJointVelocity(pybind11::object o)
{
    SetJointMatrix(&LegControllerCommand<float>::vDes, o);
}

void BaseController::SetJointTau(pybind11::object o)
{
    SetJointMatrix(&LegControllerCommand<float>::tauFeedForward, o);
}

void BaseController::SetJointForce(pybind11::object o)
{
    SetJointMatrix(&LegControllerCommand<float>::forceFeedForward, o);
}

void BaseController::SetKpJoint(pybind11::object o)
{
    SetJointMatrix(&LegControllerCommand<float>::kpJoint, o);
}

void BaseController::SetKdJoint(pybind11::object o)
{
    SetJointMatrix(&LegControllerCommand<float>::kdJoint, o);
}

void BaseController::SetKpCartesian(pybind11::object o)
{
    SetJointMatrix(&LegControllerCommand<float>::kpCartesian, o);
}

void BaseController::SetKdCartesian(pybind11::object o)
{
    SetJointMatrix(&LegControllerCommand<float>::kdCartesian, o);
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
