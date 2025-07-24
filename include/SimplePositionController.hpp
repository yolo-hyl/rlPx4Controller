#ifndef __SIMPLE_POSITION_CONTROLLER
#define __SIMPLE_POSITION_CONTROLLER
#include <Eigen/Eigen>
#include "HoverThrustEkf.hpp"
#include "Derivative.hpp"
#include <Px4ContollerParams.hpp>
enum control_mode
{
    CTRL_ALL,
    CTRL_POS_ONLY,
    CTRL_VEL_ONLY
};

double fromQuaternion2yaw(Eigen::Quaterniond q)
{
    double yaw = atan2(2 * (q.x() * q.y() + q.w() * q.z()), q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
    return yaw;
}
class SimplePositionController
{
private:
    PositionControlParams _params;
    HoverThrustEkf* hoverThrustEkf;

    HoverThrustEkf *hoverThrustEkf;
    Derivate velDerivateZ_;

    Eigen::Vector3d _pos_world;
    Eigen::Vector3d _vel_world;
    Eigen::Quaterniond _q_world;
    Eigen::Vector3d _angular_vel_world;

    double _hover_thrust{0.5}; // 不能是0

    double _thrust_sp{0.0};
    Eigen::Quaterniond q_sp;

    Eigen::Vector3d _Kp, _Kv;

    control_mode _mode = control_mode::CTRL_ALL;
    void applyVelocityConstraints(Eigen::Vector3d& vel_sp);
    void applyAccelerationConstraints(Eigen::Vector3d& acc_sp);


public:
    SimplePositionController(const PositionControlParams& params = PositionControlParams{},
        const HoverThrustEstimatorParams& hte_params = HoverThrustEstimatorParams{});

    // 参数设置接口
    void setParameters(const PositionControlParams& params);
    void setPositionGains(const Eigen::Vector3d& p_gains);
    void setVelocityGains(const Eigen::Vector3d& p_gains, const Eigen::Vector3d& i_gains, 
    const Eigen::Vector3d& d_gains);
    void setVelocityLimits(double max_horizontal, double max_up, double max_down);
    void setThrustLimits(double min_thrust, double max_thrust);
    void setHoverThrust(double hover_thrust);

    // 获取当前参数
    PositionControlParams getParameters() const { return _params; }

    // 改进的update方法，包含约束处理
    Eigen::VectorXd update(const Eigen::Vector3d& pos_sp, const Eigen::Vector3d& vel_sp, 
    const Eigen::Vector3d& acc_sp, const double yaw_sp);
    void set_pid_params(Eigen::Vector3d pos_gains, Eigen::Vector3d vel_gains)
    {
        _Kp = pos_gains;
        _Kv = vel_gains;
    };
    double get_hover_thrust()
    {
        return _hover_thrust;
    };
    void set_status(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d angular_velocity, Eigen::Vector4d q, double dt)
    {
        _vel_world = vel;
        _pos_world = pos;
        _q_world.w() = q(0);
        _q_world.x() = q(1);
        _q_world.y() = q(2);
        _q_world.z() = q(3);
        _angular_vel_world = angular_velocity;

        // _hover_thrust = hover_thrust_estimator(0.005, _vel_world(2), thrust_z(2));
        Eigen::Vector3d thrust_z = _q_world.toRotationMatrix() * Eigen::Vector3d(0, 0, _thrust_sp);

        hoverThrustEkf->predict(dt); // dt
        double acc_z = velDerivateZ_.update(_vel_world(2), dt);
        hoverThrustEkf->fuseAccZ(acc_z, thrust_z(2));
        // _hover_thrust =  hoverThrustEkf->getHoverThrust();
        _hover_thrust = 0.1533; //0.29233965277671814;

    };
    Eigen::VectorXd update(const Eigen::Vector3d &pos_sp, const Eigen::Vector3d &vel_sp, const Eigen::Vector3d &acc_sp, const double yaw_sp);
    void set_control_mode(control_mode mode);
    SimplePositionController(/* args */);
    ~SimplePositionController();
};

// 构造函数实现
SimplePositionController::SimplePositionController(const PositionControlParams& params,
    const HoverThrustEstimatorParams& hte_params) 
    : _params(params) {
    setParameters(_params);

    // 初始化悬停推力估计器
    hoverThrustEkf = new HoverThrustEkf(hte_params.initial_hover_thrust,
    hte_params.hover_thrust_noise,
    hte_params.process_noise);
    if (hte_params.enable_gate) {
        hoverThrustEkf->enableGate(hte_params.gate_size);
    }
}

void SimplePositionController::setParameters(const PositionControlParams& params) {
    _params = params;
    _Kp = _params.position_p_gain;
    _Kv = _params.velocity_p_gain;
    _hover_thrust = _params.hover_thrust;  // 移除硬编码
}

SimplePositionController::SimplePositionController(/* args */)
{
    _Kp << 1.5, 1.5, 1.5;
    _Kv << 1.5, 1.5, 1.5;
    hoverThrustEkf = new HoverThrustEkf(0.4f, 0.1f, 0.0036f);
}

SimplePositionController::~SimplePositionController()
{
}

void SimplePositionController::set_control_mode(control_mode mode)
{
    _mode = mode;
}
// 改进的update方法
Eigen::VectorXd SimplePositionController::update(const Eigen::Vector3d& pos_sp, 
    const Eigen::Vector3d& vel_sp, 
    const Eigen::Vector3d& acc_sp, 
    const double yaw_sp) {
    // 计算期望加速度
    Eigen::Vector3d des_acc = acc_sp;

    if (_mode == CTRL_POS_ONLY) {
        des_acc += _Kp.cwiseProduct(pos_sp - _pos_world);
    } else if (_mode == CTRL_VEL_ONLY) {
        des_acc += _Kv.cwiseProduct(vel_sp - _vel_world);
    } else {
        des_acc += _Kv.cwiseProduct(vel_sp - _vel_world) + 
    _   Kp.cwiseProduct(pos_sp - _pos_world);
    }

    // 应用加速度约束
    applyAccelerationConstraints(des_acc);

    // 计算推力设定值，使用动态悬停推力
    double current_hover_thrust = hoverThrustEkf->getHoverThrust();
    _thrust_sp = des_acc(2) * (current_hover_thrust / CONSTANTS_ONE_G) + current_hover_thrust;

    // 约束推力在合理范围内
    _thrust_sp = MyMath::constrain(_thrust_sp, _params.min_thrust, _params.max_thrust);

    // 计算姿态设定值（考虑倾斜角限制）
    double yaw_odom = fromQuaternion2yaw(_q_world);
    double sin_yaw = std::sin(yaw_odom);
    double cos_yaw = std::cos(yaw_odom);

    double roll = (des_acc(0) * sin_yaw - des_acc(1) * cos_yaw) / CONSTANTS_ONE_G;
    double pitch = (des_acc(0) * cos_yaw + des_acc(1) * sin_yaw) / CONSTANTS_ONE_G;

    // 限制倾斜角
    double tilt_norm = std::sqrt(roll*roll + pitch*pitch);
    if (tilt_norm > std::tan(_params.max_tilt_angle)) {
        double scale = std::tan(_params.max_tilt_angle) / tilt_norm;
        roll *= scale;
        pitch *= scale;
    }

    q_sp = Eigen::AngleAxisd(yaw_sp, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

    // 返回结果
    Eigen::VectorXd atti_thrust_sp(5);
    atti_thrust_sp << q_sp.w(), q_sp.x(), q_sp.y(), q_sp.z(), _thrust_sp;
    return atti_thrust_sp;
}
void SimplePositionController::applyAccelerationConstraints(Eigen::Vector3d& acc_sp) {
    // 水平加速度约束
    Eigen::Vector2d acc_horizontal(acc_sp(0), acc_sp(1));
    if (acc_horizontal.norm() > _params.max_horizontal_acceleration) {
        acc_horizontal = acc_horizontal.normalized() * _params.max_horizontal_acceleration;
        acc_sp(0) = acc_horizontal(0);
        acc_sp(1) = acc_horizontal(1);
    }
    
    // 垂直加速度约束
    acc_sp(2) = MyMath::constrain(acc_sp(2), 
                                 -_params.max_vertical_acceleration, 
                                 _params.max_vertical_acceleration);
}
#endif