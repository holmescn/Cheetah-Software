#ifndef PROXY_CONTROLLER_H
#define PROXY_CONTROLLER_H
#include <tuple>
#include <memory>
#include <RobotController.h>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>


class LegProxy {
    size_t _leg;
    LegController<float> *_legController;
public:
    LegProxy(LegController<float> *legCtrl, size_t leg);
    LegProxy(const LegProxy &) = delete;
    LegProxy(LegProxy &&) = delete;
    LegProxy& operator=(const LegProxy &) = delete;
    LegProxy& operator=(LegProxy &&) = delete;
    ~LegProxy() = default;

    Eigen::Vector3f PyObj2Vec3f(const pybind11::object& o);
    Eigen::Matrix3f Vec3f2Mat3f(const Eigen::Vector3f &v);

    Eigen::Vector3f GetJointAngular() const;
    void SetJointAngular(pybind11::object o);

    Eigen::Vector3f GetJointAngularVelocity() const;
    void SetJointAngularVelocity(pybind11::object o);

    Eigen::Vector3f GetJointPosition() const;
    void SetJointPosition(pybind11::object v);

    Eigen::Vector3f GetJointVelocity() const;
    void SetJointVelocity(pybind11::object o);

    Eigen::Vector3f GetJointTau() const;
    void SetJointTau(pybind11::object o);
    void SetJointForce(pybind11::object o);

    void SetKpJoint(pybind11::object v);
    void SetKdJoint(pybind11::object v);
    void SetKpCartesian(pybind11::object v);
    void SetKdCartesian(pybind11::object v);
};

class StateProxy {
    StateEstimatorContainer<float> *_stateEstimator;
public:
    StateProxy(StateEstimatorContainer<float> *estimator);
    StateProxy(const StateProxy&) = delete;
    StateProxy(StateProxy&&) = delete;
    StateProxy& operator=(const StateProxy&) = delete;
    StateProxy& operator=(StateProxy&&) = delete;
    ~StateProxy() = default;

    Eigen::Vector4f GetContact() const;
    Eigen::Vector3f GetPosition() const;
    Eigen::Vector4f GetOrientation() const;
    Eigen::Vector3f GetVBody() const;
    Eigen::Vector3f GetOmegaBody() const;
    Eigen::Matrix3f GetRBody() const;
    Eigen::Vector3f GetABody() const;
    Eigen::Vector3f GetRPY() const;
    Eigen::Vector3f GetAWorld() const;
    Eigen::Vector3f GetVWorld() const;
    Eigen::Vector3f GetOmegaWorld() const;
};

class __attribute__((visibility("hidden"))) ProxyController : public RobotController {
    pybind11::object _f_initialize, _f_run;
public:
    ProxyController(pybind11::object obj);
    virtual ~ProxyController() = default;

    void initializeController() override {
        _f_initialize();
    }

    void runController() override {
        _f_run();
    }

    void updateVisualization() override {
        // ignore this callback
    }

    inline ControlParameters* getUserControlParameters() override {
        return nullptr;
    }

    inline std::unique_ptr<LegProxy> GetLeg(size_t leg) const {
        return std::make_unique<LegProxy>(_legController, leg);
    }

    inline std::unique_ptr<StateProxy> GetState() const {
        return std::make_unique<StateProxy>(_stateEstimator);
    }

    inline void SetEnable(bool bEnable) {
        _legController->setEnabled(bEnable);
    }

    inline void SetMaxTorque(int x) {
        _legController->_maxTorque = x;
    }

    inline void SetEncodeZeros(bool bEnable) {
        _legController->_zeroEncoders = bEnable;
    }

    inline void SetCalibrate(uint32_t calibrate) {
        _legController->_calibrateEncoders = calibrate;
    }
};

#endif // PROXY_CONTROLLER_H