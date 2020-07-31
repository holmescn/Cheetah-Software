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
    void SetJointMatrix(Eigen::Vector3f LegControllerCommand<float>::* member, const Matrix43f &m);
    void SetJointMatrix(Eigen::Matrix3f LegControllerCommand<float>::* member, const Matrix43f &m);
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

    void SetJointAngular(Matrix43f m);
    void SetJointAngularVelocity(Matrix43f m);
    void SetJointPosition(Matrix43f m);
    void SetJointVelocity(Matrix43f m);
    void SetJointTau(Matrix43f m);
    void SetJointForce(Matrix43f m);
    void SetKpJoint(Matrix43f m);
    void SetKdJoint(Matrix43f m);
    void SetKpCartesian(Matrix43f m);
    void SetKdCartesian(Matrix43f m);

    std::unique_ptr<LegProperty> GetLeg() const;
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