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
        .def_readwrite("derivative_gain", &RateControlParams::derivative_gain)
        .def_readwrite("integral_limit", &RateControlParams::integral_limit)
        .def_readwrite("rate_k", &RateControlParams::rate_k);;

    py::class_<PositionControlParams>(m, "PositionControlParams")
        .def(py::init<>())
        .def_readwrite("position_p_gain", &PositionControlParams::position_p_gain)
        .def_readwrite("velocity_p_gain", &PositionControlParams::velocity_p_gain)
        .def_readwrite("velocity_i_gain", &PositionControlParams::velocity_i_gain)
        .def_readwrite("velocity_d_gain", &PositionControlParams::velocity_d_gain)
        .def_readwrite("max_horizontal_velocity", &PositionControlParams::max_horizontal_velocity)
        .def_readwrite("max_vertical_velocity_up", &PositionControlParams::max_vertical_velocity_up)
        .def_readwrite("max_vertical_velocity_down", &PositionControlParams::max_vertical_velocity_down)
        .def_readwrite("min_thrust", &PositionControlParams::min_thrust)
        .def_readwrite("max_thrust", &PositionControlParams::max_thrust)
        .def_readwrite("hover_thrust", &PositionControlParams::hover_thrust)
        .def_readwrite("max_horizontal_acceleration", &PositionControlParams::max_horizontal_acceleration)
        .def_readwrite("max_vertical_acceleration", &PositionControlParams::max_vertical_acceleration)
        .def_readwrite("max_tilt_angle", &PositionControlParams::max_tilt_angle);

    py::class_<HoverThrustEstimatorParams>(m, "HoverThrustEstimatorParams")
        .def(py::init<>())
        .def_readwrite("initial_hover_thrust", &HoverThrustEstimatorParams::initial_hover_thrust)
        .def_readwrite("hover_thrust_noise", &HoverThrustEstimatorParams::hover_thrust_noise)
        .def_readwrite("process_noise", &HoverThrustEstimatorParams::process_noise)
        .def_readwrite("acceleration_variance", &HoverThrustEstimatorParams::acceleration_variance)
        .def_readwrite("gate_size", &HoverThrustEstimatorParams::gate_size)
        .def_readwrite("enable_gate", &HoverThrustEstimatorParams::enable_gate)
        .def_readwrite("min_hover_thrust", &HoverThrustEstimatorParams::min_hover_thrust)
        .def_readwrite("max_hover_thrust", &HoverThrustEstimatorParams::max_hover_thrust);

    // 添加MixerParams绑定
    py::class_<MixerParams>(m, "MixerParams")
        .def(py::init<>())
        .def_readwrite("thrust_factor", &MixerParams::thrust_factor)
        .def_readwrite("min_output", &MixerParams::min_output)
        .def_readwrite("max_output", &MixerParams::max_output)
        .def_readwrite("max_output_yaw", &MixerParams::max_output_yaw);


        py::class_<Px4ControllerParams>(m, "Px4ControllerParams")
        .def(py::init<>())
        .def_readwrite("attitude", &Px4ControllerParams::attitude)
        .def_readwrite("rate", &Px4ControllerParams::rate)
        .def_readwrite("position", &Px4ControllerParams::position)
        .def_readwrite("hover_estimator", &Px4ControllerParams::hover_estimator)
        .def_readwrite("mixer", &Px4ControllerParams::mixer)
        .def_static("from_px4_params", &Px4ControllerParams::fromPx4Params);

    // 控制器类绑定
    py::class_<Px4AttitudeController>(m, "AttiControl")
        .def(py::init<>(), "Quaternion nonlinear attitude control, output in body coordinates")
        .def(py::init<const AttitudeControlParams&>())
        .def("set_parameters", &Px4AttitudeController::setParameters)
        .def("get_parameters", &Px4AttitudeController::getParameters)
        .def("set_pid_params", &Px4AttitudeController::set_pid_params, py::arg("p_gains").none())
        .def("update", py::overload_cast<const Eigen::Vector4d &, const Eigen::Vector4d &>(&Px4AttitudeController::update),
             py::arg("q_sp").none(), py::arg("q").none());

    py::class_<Px4RateController>(m, "RateControl")
        .def(py::init<>())
        .def(py::init<const RateControlParams&>())
        .def("set_parameters", &Px4RateController::setParameters)
        .def("get_parameters", &Px4RateController::getParameters)
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
        .def(py::init<const PositionControlParams&, const HoverThrustEstimatorParams&>())
        .def("set_parameters", &SimplePositionController::setParameters)
        .def("get_parameters", &SimplePositionController::getParameters)
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