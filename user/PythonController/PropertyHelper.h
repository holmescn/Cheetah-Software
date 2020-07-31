#ifndef PROPERTY_HELPER_H
#define PROPERTY_HELPER_H
#include <tuple>
#include <memory>
#include <RobotController.h>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>


class LegProperty {
    size_t _leg;
    LegController<float> *_legController;
public:
    LegProperty(LegController<float> *legCtrl, size_t leg);
    LegProperty(const LegProperty &) = delete;
    LegProperty(LegProperty &&) = delete;
    LegProperty& operator=(const LegProperty &) = delete;
    LegProperty& operator=(LegProperty &&) = delete;
    ~LegProperty() = default;

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

class StateProperty {
    StateEstimatorContainer<float> *_stateEstimator;
public:
    StateProperty(StateEstimatorContainer<float> *estimator);
    StateProperty(const StateProperty&) = delete;
    StateProperty(StateProperty&&) = delete;
    StateProperty& operator=(const StateProperty&) = delete;
    StateProperty& operator=(StateProperty&&) = delete;
    ~StateProperty() = default;

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

#endif // PROPERTY_HELPER_H