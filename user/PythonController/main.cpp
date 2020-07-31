/*!
 * @file main.cpp
 * @brief Main Function for the Python-binding Controller.
 *
 * The main function parses command line arguments and starts the appropriate
 * driver.
 */
#include <memory>
#include <streambuf>
#include <fstream>
#include <iostream>
#include <string>
#include <exception>

#pragma GCC diagnostic ignored "-Wshadow"
#include <pybind11/embed.h>

#include "Types.h"
#include "SimulationBridge.h"
#include "RobotController.h"
#include "ProxyController.h"
#include "BaseController.h"

namespace py = pybind11;

PYBIND11_EMBEDDED_MODULE(robot, m) {
  py::class_<BaseController, PyBaseController>(m, "BaseController")
    .def(py::init<>())
    .def("initialize", &BaseController::initialize)
    .def("run", &BaseController::run)
    .def("leg", &BaseController::GetLeg)
    .def_property_readonly("state", &BaseController::GetState)
    .def_property("enable", nullptr, &BaseController::SetEnable)
    .def_property("max_torque", nullptr, &BaseController::SetMaxTorque)
    .def_property("encode_zeros", nullptr, &BaseController::SetEncodeZeros)
    .def_property("calibrate", nullptr, &BaseController::SetCalibrate);

  py::class_<LegProxy>(m, "LegProxy")
    .def_property_readonly("q", &LegProxy::GetJointAngular)
    .def_property("desired_q", nullptr, &LegProxy::SetJointAngular)
    .def_property_readonly("dq", &LegProxy::GetJointAngularVelocity)
    .def_property("desired_dq", nullptr, &LegProxy::SetJointAngularVelocity)
    .def_property_readonly("p", &LegProxy::GetJointPosition)
    .def_property("desired_p", nullptr, &LegProxy::SetJointPosition)
    .def_property_readonly("v", &LegProxy::GetJointVelocity)
    .def_property("desired_v", nullptr, &LegProxy::SetJointVelocity)
    .def_property_readonly("tau", &LegProxy::GetJointTau)
    .def_property("tau_feed_forward", nullptr, &LegProxy::SetJointTau)
    .def_property("force_feed_forward", nullptr, &LegProxy::SetJointTau)
    .def_property("kp_joint", nullptr, &LegProxy::SetKpJoint)
    .def_property("kd_joint", nullptr, &LegProxy::SetKdJoint)
    .def_property("kp_cartesian", nullptr, &LegProxy::SetKpCartesian)
    .def_property("kd_cartesian", nullptr, &LegProxy::SetKdCartesian);

  py::class_<StateProxy>(m, "StateProxy")
    .def_property_readonly("contact", &StateProxy::GetContact)
    .def_property_readonly("position", &StateProxy::GetPosition)
    .def_property_readonly("orientation", &StateProxy::GetOrientation)
    .def_property_readonly("a_body", &StateProxy::GetABody)
    .def_property_readonly("v_body", &StateProxy::GetVBody)
    .def_property_readonly("omega_body", &StateProxy::GetOmegaBody)
    .def_property_readonly("r_body", &StateProxy::GetRBody)
    .def_property_readonly("a_world", &StateProxy::GetAWorld)
    .def_property_readonly("v_world", &StateProxy::GetVWorld)
    .def_property_readonly("omega_world", &StateProxy::GetOmegaWorld)
    .def_property_readonly("rpy", &StateProxy::GetRPY);
}

void LoadScript(const char* filename, py::object &scope)
{
  std::ifstream  ifs(filename, std::ifstream::in);
  if (!ifs) {
    char message[1024];
    sprintf(message, "Failed to read %s", filename);
    throw std::runtime_error(message);
  }

  std::string script;
  ifs.seekg(0, std::ios::end);   
  script.reserve(ifs.tellg());
  ifs.seekg(0, std::ios::beg);
  script.assign(std::istreambuf_iterator<char>(ifs),
                std::istreambuf_iterator<char>());

  py::exec(script.c_str(), scope);
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cout << "USAGE: " << argv[0] << " <py_file>" << std::endl;
    return EXIT_FAILURE;
  }

  py::scoped_interpreter guard{};

  // Evaluate in scope of main module
  py::object scope = py::module::import("__main__").attr("__dict__");
  LoadScript(argv[1], scope);

  std::string robotMode = "m";
  if (scope.contains("ROBOT_MODE")) {
    robotMode = scope["ROBOT_MODE"].cast<std::string>();
  }

  std::string controllerClassName;
  if (scope.contains("CONTROLLER_CLASS_NAME")) {
    controllerClassName = scope["CONTROLLER_CLASS_NAME"].cast<std::string>();
  } else {
    std::cerr << "CONTROLLER_CLASS_NAME is not specified." << std::endl;
    return EXIT_FAILURE;
  }

  MasterConfig cfg;
  if (robotMode == "3") {
    cfg._robot = RobotType::CHEETAH_3;
  } else if (robotMode == "m") {
    cfg._robot = RobotType::MINI_CHEETAH;
  } else {
    std::cerr << "Invalid ROBOT_MODE: " << robotMode << std::endl;
    return EXIT_FAILURE;
  }

  // Python-binding works for simulator only.
  cfg.simulated = true;
  cfg.load_from_file = false;

  std::cout << "[Quadruped] Cheetah Software" << std::endl;
  std::cout << "Quadruped: "
            << (cfg._robot == RobotType::MINI_CHEETAH ?
                "Mini Cheetah" : "Cheetah 3") << std::endl;
  printf("   Driver: %s\n", cfg.simulated ? "Development Simulation Driver" : "Quadruped Driver");

  // Create controller
  puts("[DEBUG] Create global controller instance");
  py::exec("g_controller = " + controllerClassName + "()", scope);
  puts("[DEBUG] Fetch global controller instance");
  std::unique_ptr<ProxyController> proxy = std::make_unique<ProxyController>(scope["g_controller"]);

  // dispatch the appropriate driver
  if (cfg._robot == RobotType::MINI_CHEETAH) {
    SimulationBridge simulationBridge(cfg._robot, proxy.get());
    simulationBridge.run();
  } else if (cfg._robot == RobotType::CHEETAH_3) {
    SimulationBridge simulationBridge(cfg._robot, proxy.get());
    simulationBridge.run();
  } else {
    puts("[ERROR] unknown robot mode");
    return EXIT_FAILURE;
  }

  puts("[Quadruped] SimDriver run() has finished!");
  return EXIT_SUCCESS;
}
