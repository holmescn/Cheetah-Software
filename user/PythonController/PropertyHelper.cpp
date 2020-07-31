#pragma GCC diagnostic ignored "-Wshadow"

#include <cstdio>
#include "BaseController.h"
#include "PropertyHelper.h"

namespace py = pybind11;

LegProperty::LegProperty(LegController<float> *legCtrl, size_t leg)
: _leg(leg), _legController(legCtrl)
{
  //  
}

Eigen::Matrix3f LegProperty::Vec3f2Mat3f(const Eigen::Vector3f &v)
{
  Eigen::Matrix3f m = Eigen::Matrix3f::Zero();
  m(0, 0) = v[0];
  m(1, 1) = v[1];
  m(2, 2) = v[2];
  return m;
}

Eigen::Vector3f LegProperty::PyObj2Vec3f(const pybind11::object& o)
{
  Eigen::Vector3f v = Eigen::Vector3f::Zero();
  if (py::isinstance<py::float_>(o)) {
    float f = o.cast<float>();
    v << f, f, f;
  } else if (py::isinstance<py::tuple>(o) || py::isinstance<py::list>(o)) {
    size_t i = 0;
    for(auto it = o.begin(); it != o.end() && i < 3; ++it, ++i) {
      v[i] = it->cast<float>();
    }
  } else {
    v = o.cast<Eigen::Vector3f>();
  }

  return v;
}

Eigen::Vector3f LegProperty::GetJointAngular() const
{
  return _legController ? _legController->datas[_leg].q : Eigen::Vector3f::Zero();
}

void LegProperty::SetJointAngular(pybind11::object o)
{
  if (_legController) {
    _legController->commands[_leg].qDes = PyObj2Vec3f(o);
  }
}

Eigen::Vector3f LegProperty::GetJointAngularVelocity() const
{
  return _legController ? _legController->datas[_leg].qd : Eigen::Vector3f::Zero();
}

void LegProperty::SetJointAngularVelocity(py::object o)
{
  if (_legController) {
    _legController->commands[_leg].qdDes = PyObj2Vec3f(o);
  }
}

Eigen::Vector3f LegProperty::GetJointPosition() const
{
  return _legController ? _legController->datas[_leg].p : Eigen::Vector3f::Zero();
}

void LegProperty::SetJointPosition(py::object o)
{
  if (_legController) {
    _legController->commands[_leg].pDes = PyObj2Vec3f(o);
  }
}

Eigen::Vector3f LegProperty::GetJointVelocity() const
{
  return _legController ? _legController->datas[_leg].v : Eigen::Vector3f::Zero();
}

void LegProperty::SetJointVelocity(py::object o)
{
  if (_legController) {
    _legController->commands[_leg].vDes = PyObj2Vec3f(o);
  }
}

Eigen::Vector3f LegProperty::GetJointTau() const
{
  return _legController ? _legController->datas[_leg].tauEstimate : Eigen::Vector3f::Zero();
}

void LegProperty::SetJointTau(py::object o)
{
  if (_legController) {
    _legController->commands[_leg].tauFeedForward = PyObj2Vec3f(o);
  }
}

void LegProperty::SetJointForce(py::object o)
{
  if (_legController) {
    _legController->commands[_leg].forceFeedForward = PyObj2Vec3f(o);
  }
}

void LegProperty::SetKpJoint(py::object o)
{
  if (_legController) {
    _legController->commands[_leg].kpJoint = Vec3f2Mat3f(PyObj2Vec3f(o));
  }
}

void LegProperty::SetKdJoint(py::object o)
{
  if (_legController) {
    _legController->commands[_leg].kdJoint = Vec3f2Mat3f(PyObj2Vec3f(o));
  }
}

void LegProperty::SetKpCartesian(py::object o)
{
  if (_legController) {
    _legController->commands[_leg].kpCartesian = Vec3f2Mat3f(PyObj2Vec3f(o));
  }
}

void LegProperty::SetKdCartesian(py::object o)
{
  if (_legController) {
    _legController->commands[_leg].kdCartesian = Vec3f2Mat3f(PyObj2Vec3f(o));
  }
}

StateProperty::StateProperty(StateEstimatorContainer<float> *estimator)
: _stateEstimator(estimator)
{

}

Eigen::Vector4f StateProperty::GetContact() const
{
  if (_stateEstimator) {
    auto r = _stateEstimator->getResult();
    return r.contactEstimate;
  }
  return Eigen::Vector4f::Zero();
}

Eigen::Vector3f StateProperty::GetPosition() const
{
  if (_stateEstimator) {
    auto r = _stateEstimator->getResult();
    return r.position;
  }
  return Eigen::Vector3f::Zero();
}
  
Eigen::Vector4f StateProperty::GetOrientation() const
{
  if (_stateEstimator) {
    auto r = _stateEstimator->getResult();
    return r.orientation;
  }
  return Eigen::Vector4f::Zero();
}

Eigen::Vector3f StateProperty::GetVBody() const
{
  if (_stateEstimator) {
    auto r = _stateEstimator->getResult();
    return r.vBody;
  }
  return Eigen::Vector3f::Zero();
}

Eigen::Vector3f StateProperty::GetOmegaBody() const
{
  if (_stateEstimator) {
    auto r = _stateEstimator->getResult();
    return r.omegaBody;
  }
  return Eigen::Vector3f::Zero();
}

Eigen::Matrix3f StateProperty::GetRBody() const
{
  if (_stateEstimator) {
    auto r = _stateEstimator->getResult();
    return r.rBody;
  }
  return Eigen::Matrix3f::Zero();
}

Eigen::Vector3f StateProperty::GetABody() const
{
  if (_stateEstimator) {
    auto r = _stateEstimator->getResult();
    return r.aBody;
  }
  return Eigen::Vector3f::Zero();
}

Eigen::Vector3f StateProperty::GetRPY() const
{
  if (_stateEstimator) {
    auto r = _stateEstimator->getResult();
    return r.rpy;
  }
  return Eigen::Vector3f::Zero();
}

Eigen::Vector3f StateProperty::GetAWorld() const
{
  if (_stateEstimator) {
    auto r = _stateEstimator->getResult();
    return r.aWorld;
  }
  return Eigen::Vector3f::Zero();
}

Eigen::Vector3f StateProperty::GetVWorld() const
{
  if (_stateEstimator) {
    auto r = _stateEstimator->getResult();
    return r.vWorld;
  }
  return Eigen::Vector3f::Zero();
}

Eigen::Vector3f StateProperty::GetOmegaWorld() const
{
  if (_stateEstimator) {
    auto r = _stateEstimator->getResult();
    return r.omegaWorld;
  }
  return Eigen::Vector3f::Zero();
}
