#pragma GCC diagnostic ignored "-Wshadow"

#include <pybind11/pybind11.h>
#include "BaseController.h"
#include "ProxyController.h"

void BaseController::SetEnable(bool bEnable)
{
    _proxy->SetEnable(bEnable);
}

void BaseController::SetMaxTorque(int x)
{
    _proxy->SetMaxTorque(x);
}

void BaseController::SetEncodeZeros(bool bEnable)
{
    _proxy->SetEncodeZeros(bEnable);
}

void BaseController::SetCalibrate(uint32_t calibrate)
{
    _proxy->SetCalibrate(calibrate);
}

std::unique_ptr<LegProxy> BaseController::GetLeg(size_t leg) const
{
    if (leg > 3) {
        throw std::invalid_argument("leg should be in [0, 3].");
    }
    return _proxy->GetLeg(leg);
}

std::unique_ptr<StateProxy> BaseController::GetState() const
{
    return _proxy->GetState();
}

Eigen::Matrix<float, 4, 3> BaseController::GetJointAngular() const
{

}

Eigen::Matrix<float, 4, 3> BaseController::GetJointAngularVelocity() const
{

}

Eigen::Matrix<float, 4, 3> BaseController::GetJointPosition() const
{

}

Eigen::Matrix<float, 4, 3> BaseController::GetJointVelocity() const
{
    
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
