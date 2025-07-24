#ifndef __PX4_CONTROLLER_PARAMS_HPP_
#define __PX4_CONTROLLER_PARAMS_HPP_

#include <Eigen/Eigen>

struct AttitudeControlParams {
    Eigen::Vector3d proportional_gain{6.5, 6.5, 2.8};  // PX4默认值
    Eigen::Vector3d rate_limit{1600.0/57.3, 1600.0/57.3, 1000.0/57.3}; // rad/s
    double yaw_weight = 0.4;
    double yawspeed_setpoint = 0.0;
};

struct RateControlParams {
    // PX4官方默认参数
    Eigen::Vector3d proportional_gain{0.15, 0.15, 0.2};
    Eigen::Vector3d integral_gain{0.2, 0.2, 0.2}; 
    Eigen::Vector3d derivative_gain{0.003, 0.003, 0.0};
    Eigen::Vector3d integral_limit{0.3, 0.3, 0.3};
    Eigen::Vector3d rate_k{1.0, 1.0, 1.0};  // 全局增益
    double derivative_cutoff_freq = 20.0;
};

struct PositionControlParams {
    // 位置控制增益
    Eigen::Vector3d position_p_gain{1.0, 1.0, 1.0};
    Eigen::Vector3d velocity_p_gain{4.0, 4.0, 4.0};
    Eigen::Vector3d velocity_i_gain{2.0, 2.0, 2.0};
    Eigen::Vector3d velocity_d_gain{0.0, 0.0, 0.0};
    
    // 速度限制
    double max_horizontal_velocity = 12.0;  // m/s
    double max_vertical_velocity_up = 3.0;
    double max_vertical_velocity_down = 1.0;
    
    // 推力限制
    double min_thrust = 0.02;
    double max_thrust = 1.0;
    double hover_thrust = 0.5;  // 可配置的悬停推力
    
    // 加速度限制
    double max_horizontal_acceleration = 5.0;  // m/s²
    double max_vertical_acceleration = 9.0;
    
    // 倾斜角限制
    double max_tilt_angle = 0.7854;  // 45度，单位弧度
};

struct HoverThrustEstimatorParams {
    double initial_hover_thrust = 0.5;
    double hover_thrust_noise = 0.1;
    double process_noise = 0.0036;
    double acceleration_variance = 5.0;
    double gate_size = 3.0;
    bool enable_gate = false;
    double min_hover_thrust = 0.1;
    double max_hover_thrust = 0.9;
};

struct MixerParams {
    double thrust_factor = 1.0;  // 推力模型参数
    double min_output = 0.0;
    double max_output = 1.0;
    double max_output_yaw = 1.15;  // yaw通道允许稍微超出范围
};

// 完整的控制器参数集合
struct Px4ControllerParams {
    AttitudeControlParams attitude;
    RateControlParams rate;
    PositionControlParams position;
    HoverThrustEstimatorParams hover_estimator;
    MixerParams mixer;
    
    // 从PX4参数文件加载
    static Px4ControllerParams fromPx4Params(const std::string& param_file_path);
    
    // 保存到文件
    void saveToFile(const std::string& file_path) const;
    
    // 从文件加载
    static Px4ControllerParams loadFromFile(const std::string& file_path);
};

#endif
