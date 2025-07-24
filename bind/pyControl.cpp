#include "Px4AttitudeController.hpp"
#include "Px4RateController.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "SimplePositionController.hpp"
#include "Px4Mixer.hpp"

namespace py = pybind11;

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

PYBIND11_MODULE(pyControl, m)
{
    m.doc() = R"pbdoc(
        Quadrotor Controller Utils
        -----------------------
    )pbdoc";
    py::class_<AttitudeControlParams>(m, "AttitudeControlParams")
        .def(py::init<>())
        .def_readwrite("proportional_gain", &AttitudeControlParams::proportional_gain)
        .def_readwrite("rate_limit", &AttitudeControlParams::rate_limit)
        .def_readwrite("yaw_weight", &AttitudeControlParams::yaw_weight);

    py::class_<RateControlParams>(m, "RateControlParams")
        .def(py::init<>())
        .def_readwrite("proportional_gain", &RateControlParams::proportional_gain)
        .def_readwrite("integral_gain", &RateControlParams::integral_gain)
        .def_readwrite("derivative_gain", &RateControlParams::derivative_gain);

    py::class_<PositionControlParams>(m, "PositionControlParams")
        .def(py::init<>())
        .def_readwrite("position_p_gain", &PositionControlParams::position_p_gain)
        .def_readwrite("velocity_p_gain", &PositionControlParams::velocity_p_gain)
        .def_readwrite("hover_thrust", &PositionControlParams::hover_thrust);

    py::class_<Px4ControllerParams>(m, "Px4ControllerParams")
        .def(py::init<>())
        .def_readwrite("attitude", &Px4ControllerParams::attitude)
        .def_readwrite("rate", &Px4ControllerParams::rate)
        .def_readwrite("position", &Px4ControllerParams::position)
        .def_static("from_px4_params", &Px4ControllerParams::fromPx4Params);

    // 更新控制器类，添加参数构造函数
    py::class_<Px4AttitudeController>(m, "AttiControl")
        .def(py::init<>())
        .def(py::init<const AttitudeControlParams&>())
        .def("set_parameters", &Px4AttitudeController::setParameters)
        .def("get_parameters", &Px4AttitudeController::getParameters)
        .def("update", py::overload_cast<const Eigen::Vector4d &, const Eigen::Vector4d &>(&Px4AttitudeController::update));

    py::class_<Px4AttitudeController>(m, "AttiControl")
        .def(py::init<>(), "Quaternion nonlinear attitude control, output in body coordinates")
        .def("set_pid_params", &Px4AttitudeController::set_pid_params, py::arg("p_gains").none())
        .def("update", py::overload_cast<const Eigen::Vector4d &, const Eigen::Vector4d &>(&Px4AttitudeController::update),
             py::arg("q_sp").none(), py::arg("q").none());

    py::class_<Px4RateController>(m, "RateControl")
        .def(py::init<>())
        .def("set_q_world", &Px4RateController::set_q_world, py::arg("q_world").none())
        .def("set_pid_params", &Px4RateController::set_pid_params, py::arg("p_gains").none(), py::arg("i_gains").none(), py::arg("d_gains").none())
        .def("update", py::overload_cast<const Eigen::Vector3d &, const Eigen::Vector3d &, const Eigen::Vector3d &, const float>(&Px4RateController::update),
             py::arg("rate_sp").none(), py::arg("rate").none(), py::arg("angular_accel").none(), py::arg("dt").none());

    py::class_<Px4Mixer>(m, "Mixer")
        .def(py::init<>())
        .def("update", py::overload_cast<const Eigen::Vector4d &>(&Px4Mixer::update),
             py::arg("torque").none());

    py::class_<SimplePositionController>(m, "PosControl")
        .def(py::init<>())
        .def("get_hover_thrust", &SimplePositionController::get_hover_thrust)
        .def("set_pid_params", &SimplePositionController::set_pid_params, py::arg("pos_gains").none(), py::arg("vel_gains").none())
        .def("set_status", &SimplePositionController::set_status, py::arg("pos").none(), py::arg("vel").none(), py::arg("angular_velocity").none(), py::arg("q").none(), py::arg("dt").none())
        .def("update", py::overload_cast<const Eigen::Vector3d &, const Eigen::Vector3d &, const Eigen::Vector3d &, const double>(&SimplePositionController::update),
             py::arg("pos_sp").none(), py::arg("vel_sp").none(), py::arg("acc_sp").none(), py::arg("yaw_sp").none());

    ;
#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}