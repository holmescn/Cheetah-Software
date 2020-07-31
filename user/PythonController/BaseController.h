#ifndef BASE_CONTROLLER_H
#define BASE_CONTROLLER_H
#include <memory>
#include <pybind11/pybind11.h>
#include <RobotController.h>

class LegProperty;
class StateProperty;

class BaseController : public RobotController {
    pybind11::object _py_instance;
    pybind11::object _fn_initialize, _fn_run;
public:

    BaseController();
    ~BaseController() = default;

    virtual void initialize() = 0;
    virtual void run() = 0;

    void initializeController() override;
    void runController() override;
    void updateVisualization() override;
    ControlParameters* getUserControlParameters() override;

    void SetPyInstance(pybind11::object o);
    void SetEnable(bool bEnable);
    void SetEncodeZeros(bool bEnable);
    void SetMaxTorque(float maxTorque);
    void SetCalibrate(uint32_t calibrate);

    Eigen::Matrix<float, 4, 3> GetJointAngular() const;
    Eigen::Matrix<float, 4, 3> GetJointAngularVelocity() const;
    Eigen::Matrix<float, 4, 3> GetJointPosition() const;
    Eigen::Matrix<float, 4, 3> GetJointVelocity() const;

    std::unique_ptr<LegProperty> GetLeg(size_t leg) const;
    std::unique_ptr<StateProperty> GetState() const;
};

class PyBaseController : public BaseController {
public:
    using BaseController::BaseController;

    void initialize() override;
    void run() override;

    void initializeController() override;
    void runController() override;
    void updateVisualization() override;
    ControlParameters* getUserControlParameters() override;
};

#endif // BASE_CONTROLLER_H