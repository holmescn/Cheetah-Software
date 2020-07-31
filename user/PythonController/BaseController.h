#ifndef BASE_CONTROLLER_H
#define BASE_CONTROLLER_H
#include <memory>

class ProxyController;
class LegProxy;
class StateProxy;

class BaseController {
    ProxyController *_proxy;
public:
    BaseController() = default;
    ~BaseController() = default;

    virtual void initialize() = 0;
    virtual void run() = 0;

    inline void SetProxy(ProxyController *proxy) { _proxy = proxy; }
    void SetEnable(bool bEnable);
    void SetEncodeZeros(bool bEnable);
    void SetMaxTorque(int maxTorque);
    void SetCalibrate(uint32_t calibrate);

    Eigen::Matrix<float, 4, 3> GetJointAngular() const;
    Eigen::Matrix<float, 4, 3> GetJointAngularVelocity() const;
    Eigen::Matrix<float, 4, 3> GetJointPosition() const;
    Eigen::Matrix<float, 4, 3> GetJointVelocity() const;

    std::unique_ptr<LegProxy> GetLeg(size_t leg) const;
    std::unique_ptr<StateProxy> GetState() const;
};

class PyBaseController : public BaseController {
public:
    using BaseController::BaseController;

    void initialize() override;
    void run() override;
};

#endif // BASE_CONTROLLER_H