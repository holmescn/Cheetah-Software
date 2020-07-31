#pragma GCC diagnostic ignored "-Wshadow"

#include <cstdio>
#include "BaseController.h"
#include "PropertyHelper.h"

namespace py = pybind11;

ProxyController::ProxyController(pybind11::object obj)
{
  BaseController *base = obj.cast<BaseController*>();
  base->SetProxy(this);

  _f_run = obj.attr("run");
  _f_initialize = obj.attr("initialize");
}

LegProxy::LegProxy(LegController<float> *legCtrl, size_t leg)
: _leg(leg), _legController(legCtrl)
{
  //  
}

Eigen::Matrix3f LegProxy::Vec3f2Mat3f(const Eigen::Vector3f &v)
{
  Eigen::Matrix3f m = Eigen::Matrix3f::Zero();
  m(0, 0) = v[0];
  m(1, 1) = v[1];
  m(2, 2) = v[2];
  return m;
}

Eigen::Vector3f LegProxy::PyObj2Vec3f(const pybind11::object& o)
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

Eigen::Vector3f LegProxy::GetJointAngular() const
{
  return _legController ? _legController->datas[_leg].q : Eigen::Vector3f::Zero();
}

void LegProxy::SetJointAngular(pybind11::object o)
{
  if (_legController) {
    _legController->commands[_leg].qDes = PyObj2Vec3f(o);
  }
}

Eigen::Vector3f LegProxy::GetJointAngularVelocity() const
{
  return _legController ? _legController->datas[_leg].qd : Eigen::Vector3f::Zero();
}

void LegProxy::SetJointAngularVelocity(py::object o)
{
  if (_legController) {
    _legController->commands[_leg].qdDes = PyObj2Vec3f(o);
  }
}

Eigen::Vector3f LegProxy::GetJointPosition() const
{
  return _legController ? _legController->datas[_leg].p : Eigen::Vector3f::Zero();
}

void LegProxy::SetJointPosition(py::object o)
{
  if (_legController) {
    _legController->commands[_leg].pDes = PyObj2Vec3f(o);
  }
}

Eigen::Vector3f LegProxy::GetJointVelocity() const
{
  return _legController ? _legController->datas[_leg].v : Eigen::Vector3f::Zero();
}

void LegProxy::SetJointVelocity(py::object o)
{
  if (_legController) {
    _legController->commands[_leg].vDes = PyObj2Vec3f(o);
  }
}

Eigen::Vector3f LegProxy::GetJointTau() const
{
  return _legController ? _legController->datas[_leg].tauEstimate : Eigen::Vector3f::Zero();
}

void LegProxy::SetJointTau(py::object o)
{
  if (_legController) {
    _legController->commands[_leg].tauFeedForward = PyObj2Vec3f(o);
  }
}

void LegProxy::SetJointForce(py::object o)
{
  if (_legController) {
    _legController->commands[_leg].forceFeedForward = PyObj2Vec3f(o);
  }
}

void LegProxy::SetKpJoint(py::object o)
{
  if (_legController) {
    _legController->commands[_leg].kpJoint = Vec3f2Mat3f(PyObj2Vec3f(o));
  }
}

void LegProxy::SetKdJoint(py::object o)
{
  if (_legController) {
    _legController->commands[_leg].kdJoint = Vec3f2Mat3f(PyObj2Vec3f(o));
  }
}

void LegProxy::SetKpCartesian(py::object o)
{
  if (_legController) {
    _legController->commands[_leg].kpCartesian = Vec3f2Mat3f(PyObj2Vec3f(o));
  }
}

void LegProxy::SetKdCartesian(py::object o)
{
  if (_legController) {
    _legController->commands[_leg].kdCartesian = Vec3f2Mat3f(PyObj2Vec3f(o));
  }
}

StateProxy::StateProxy(StateEstimatorContainer<float> *estimator)
: _stateEstimator(estimator)
{

}

Eigen::Vector4f StateProxy::GetContact() const
{
  if (_stateEstimator) {
    auto r = _stateEstimator->getResult();
    return r.contactEstimate;
  }
  return Eigen::Vector4f::Zero();
}

Eigen::Vector3f StateProxy::GetPosition() const
{
  if (_stateEstimator) {
    auto r = _stateEstimator->getResult();
    return r.position;
  }
  return Eigen::Vector3f::Zero();
}
  
Eigen::Vector4f StateProxy::GetOrientation() const
{
  if (_stateEstimator) {
    auto r = _stateEstimator->getResult();
    return r.orientation;
  }
  return Eigen::Vector4f::Zero();
}

Eigen::Vector3f StateProxy::GetVBody() const
{
  if (_stateEstimator) {
    auto r = _stateEstimator->getResult();
    return r.vBody;
  }
  return Eigen::Vector3f::Zero();
}

Eigen::Vector3f StateProxy::GetOmegaBody() const
{
  if (_stateEstimator) {
    auto r = _stateEstimator->getResult();
    return r.omegaBody;
  }
  return Eigen::Vector3f::Zero();
}

Eigen::Matrix3f StateProxy::GetRBody() const
{
  if (_stateEstimator) {
    auto r = _stateEstimator->getResult();
    return r.rBody;
  }
  return Eigen::Matrix3f::Zero();
}

Eigen::Vector3f StateProxy::GetABody() const
{
  if (_stateEstimator) {
    auto r = _stateEstimator->getResult();
    return r.aBody;
  }
  return Eigen::Vector3f::Zero();
}

Eigen::Vector3f StateProxy::GetRPY() const
{
  if (_stateEstimator) {
    auto r = _stateEstimator->getResult();
    return r.rpy;
  }
  return Eigen::Vector3f::Zero();
}

Eigen::Vector3f StateProxy::GetAWorld() const
{
  if (_stateEstimator) {
    auto r = _stateEstimator->getResult();
    return r.aWorld;
  }
  return Eigen::Vector3f::Zero();
}

Eigen::Vector3f StateProxy::GetVWorld() const
{
  if (_stateEstimator) {
    auto r = _stateEstimator->getResult();
    return r.vWorld;
  }
  return Eigen::Vector3f::Zero();
}

Eigen::Vector3f StateProxy::GetOmegaWorld() const
{
  if (_stateEstimator) {
    auto r = _stateEstimator->getResult();
    return r.omegaWorld;
  }
  return Eigen::Vector3f::Zero();
}
