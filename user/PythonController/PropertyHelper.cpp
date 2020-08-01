#pragma GCC diagnostic ignored "-Wshadow"

#include <cstdio>
#include "BaseController.h"
#include "PropertyHelper.h"

namespace py = pybind11;


static Eigen::Matrix3f Vec3fToMat3f(const Eigen::Vector3f &v)
{
  Eigen::Matrix3f m = Eigen::Matrix3f::Zero();
  m(0, 0) = v[0];
  m(1, 1) = v[1];
  m(2, 2) = v[2];
  return m;
}

static Eigen::Vector3f PyObjToVec3f(const pybind11::object& o)
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

LegProperty::LegProperty(LegController<float> *legCtrl)
: _leg(0), _legController(legCtrl)
{
  //  
}

Eigen::Vector3f LegProperty::GetJointVec(Eigen::Vector3f LegControllerData<float>::* member) const
{
  return _legController ? (_legController->datas[_leg].*member) : Eigen::Vector3f::Zero();
}

void LegProperty::SetJointVec(Eigen::Vector3f LegControllerCommand<float>::* member, pybind11::object &o)
{
  if (_legController) {
    _legController->commands[_leg].*member = PyObjToVec3f(o);
  }
}

void LegProperty::SetJointMat(Eigen::Matrix3f LegControllerCommand<float>::* member, pybind11::object &o)
{
  if (_legController) {
    _legController->commands[_leg].*member = Vec3fToMat3f(PyObjToVec3f(o));
  }
}

Eigen::Vector3f LegProperty::GetJointAngular() const
{
  return GetJointVec(&LegControllerData<float>::q);
}

void LegProperty::SetJointAngular(pybind11::object o)
{
  SetJointVec(&LegControllerCommand<float>::qDes, o);
}

Eigen::Vector3f LegProperty::GetJointAngularVelocity() const
{
  return GetJointVec(&LegControllerData<float>::qd);
}

void LegProperty::SetJointAngularVelocity(py::object o)
{
  SetJointVec(&LegControllerCommand<float>::qdDes, o);
}

Eigen::Vector3f LegProperty::GetJointPosition() const
{
  return GetJointVec(&LegControllerData<float>::p);
}

void LegProperty::SetJointPosition(py::object o)
{
  SetJointVec(&LegControllerCommand<float>::pDes, o);
}

Eigen::Vector3f LegProperty::GetJointVelocity() const
{
  return GetJointVec(&LegControllerData<float>::v);
}

void LegProperty::SetJointVelocity(py::object o)
{
  SetJointVec(&LegControllerCommand<float>::vDes, o);
}

Eigen::Vector3f LegProperty::GetJointTau() const
{
  return GetJointVec(&LegControllerData<float>::tauEstimate);
}

void LegProperty::SetJointTau(py::object o)
{
  SetJointVec(&LegControllerCommand<float>::tauFeedForward, o);
}

void LegProperty::SetJointForce(py::object o)
{
  SetJointVec(&LegControllerCommand<float>::forceFeedForward, o);
}

void LegProperty::SetKpJoint(py::object o)
{
  SetJointMat(&LegControllerCommand<float>::kpJoint, o);
}

void LegProperty::SetKdJoint(py::object o)
{
  SetJointMat(&LegControllerCommand<float>::kdJoint, o);
}

void LegProperty::SetKpCartesian(py::object o)
{
  SetJointMat(&LegControllerCommand<float>::kpCartesian, o);
}

void LegProperty::SetKdCartesian(py::object o)
{
  SetJointMat(&LegControllerCommand<float>::kdCartesian, o);
}

StateProperty::StateProperty(StateEstimatorContainer<float> *estimator)
: _stateEstimator(estimator)
{

}

Eigen::Vector4f StateProperty::GetContact() const
{
  return GetStateVec3f(&StateEstimate<float>::contactEstimate);
}

Eigen::Vector3f StateProperty::GetPosition() const
{
  return GetStateVec3f(&StateEstimate<float>::position);
}

Eigen::Vector4f StateProperty::GetOrientation() const
{
  return GetStateVec3f(&StateEstimate<float>::orientation);
}

Eigen::Vector3f StateProperty::GetVBody() const
{
  return GetStateVec3f(&StateEstimate<float>::vBody);
}

Eigen::Vector3f StateProperty::GetOmegaBody() const
{
  return GetStateVec3f(&StateEstimate<float>::omegaBody);
}

Eigen::Matrix3f StateProperty::GetRBody() const
{
  return GetStateVec3f(&StateEstimate<float>::rBody);
}

Eigen::Vector3f StateProperty::GetABody() const
{
  return GetStateVec3f(&StateEstimate<float>::aBody);
}

Eigen::Vector3f StateProperty::GetRPY() const
{
  return GetStateVec3f(&StateEstimate<float>::rpy);
}

Eigen::Vector3f StateProperty::GetAWorld() const
{
  return GetStateVec3f(&StateEstimate<float>::aWorld);
}

Eigen::Vector3f StateProperty::GetVWorld() const
{
  return GetStateVec3f(&StateEstimate<float>::vWorld);
}

Eigen::Vector3f StateProperty::GetOmegaWorld() const
{
  return GetStateVec3f(&StateEstimate<float>::omegaWorld);
}
