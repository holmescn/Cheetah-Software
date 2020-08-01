#ifndef BASE_CONTROLLER_H
#define BASE_CONTROLLER_H
#include <memory>
#include <pybind11/pybind11.h>
#include <RobotController.h>

class LegProperty;
class StateProperty;

using Matrix43f = Eigen::Matrix<float, 4, 3>;

class BaseController : public RobotController {
    pybind11::object _py_instance;
    pybind11::object _fn_initialize, _fn_run;
    Matrix43f GetJointMatrix(Eigen::Vector3f LegControllerData<float>::* member) const;
    void SetJointMatrix(Eigen::Vector3f LegControllerCommand<float>::* member, const pybind11::object &o);
    void SetJointMatrix(Eigen::Matrix3f LegControllerCommand<float>::* member, const pybind11::object &o);
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

    Matrix43f GetJointAngular() const;
    Matrix43f GetJointAngularVelocity() const;
    Matrix43f GetJointPosition() const;
    Matrix43f GetJointVelocity() const;

    void SetJointAngular(pybind11::object o);
    void SetJointAngularVelocity(pybind11::object o);
    void SetJointPosition(pybind11::object o);
    void SetJointVelocity(pybind11::object o);
    void SetJointTau(pybind11::object o);
    void SetJointForce(pybind11::object o);
    void SetKpJoint(pybind11::object o);
    void SetKdJoint(pybind11::object o);
    void SetKpCartesian(pybind11::object o);
    void SetKdCartesian(pybind11::object o);

    std::unique_ptr<LegProperty> GetLeg() const;
    std::unique_ptr<StateProperty> GetState() const;
};

class PyBaseController : public BaseController {
public:
    using BaseController::BaseController;

    void initialize() override;
    void run() override;
};

#endif // BASE_CONTROLLER_H