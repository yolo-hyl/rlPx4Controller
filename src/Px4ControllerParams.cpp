#include "Px4ControllerParams.hpp"
#include <fstream>
#include <sstream>
#include <iostream>

Px4ControllerParams Px4ControllerParams::fromPx4Params(const std::string& param_file_path) {
    Px4ControllerParams params;
    std::ifstream file(param_file_path);
    std::string line;
    
    if (!file.is_open()) {
        std::cerr << "Warning: Cannot open PX4 param file " << param_file_path 
                  << ", using default values" << std::endl;
        return params;
    }
    
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        
        std::istringstream iss(line);
        std::string param_name;
        double value;
        
        if (iss >> param_name >> value) {
            // 姿态控制参数
            if (param_name == "MC_ROLL_P") params.attitude.proportional_gain(0) = value;
            else if (param_name == "MC_PITCH_P") params.attitude.proportional_gain(1) = value;
            else if (param_name == "MC_YAW_P") params.attitude.proportional_gain(2) = value;
            else if (param_name == "MC_YAW_WEIGHT") params.attitude.yaw_weight = value;
            
            // 角速度控制参数  
            else if (param_name == "MC_ROLLRATE_P") params.rate.proportional_gain(0) = value;
            else if (param_name == "MC_PITCHRATE_P") params.rate.proportional_gain(1) = value;
            else if (param_name == "MC_YAWRATE_P") params.rate.proportional_gain(2) = value;
            
            else if (param_name == "MC_ROLLRATE_I") params.rate.integral_gain(0) = value;
            else if (param_name == "MC_PITCHRATE_I") params.rate.integral_gain(1) = value;
            else if (param_name == "MC_YAWRATE_I") params.rate.integral_gain(2) = value;
            
            else if (param_name == "MC_ROLLRATE_D") params.rate.derivative_gain(0) = value;
            else if (param_name == "MC_PITCHRATE_D") params.rate.derivative_gain(1) = value;
            else if (param_name == "MC_YAWRATE_D") params.rate.derivative_gain(2) = value;
            
            else if (param_name == "MC_ROLLRATE_K") params.rate.rate_k(0) = value;
            else if (param_name == "MC_PITCHRATE_K") params.rate.rate_k(1) = value;
            else if (param_name == "MC_YAWRATE_K") params.rate.rate_k(2) = value;
            
            // 位置控制参数
            else if (param_name == "MPC_XY_P") {
                params.position.position_p_gain(0) = value;
                params.position.position_p_gain(1) = value;
            }
            else if (param_name == "MPC_Z_P") params.position.position_p_gain(2) = value;
            
            else if (param_name == "MPC_XY_VEL_P") {
                params.position.velocity_p_gain(0) = value;
                params.position.velocity_p_gain(1) = value;
            }
            else if (param_name == "MPC_Z_VEL_P") params.position.velocity_p_gain(2) = value;
            
            else if (param_name == "MPC_XY_VEL_I") {
                params.position.velocity_i_gain(0) = value;
                params.position.velocity_i_gain(1) = value;
            }
            else if (param_name == "MPC_Z_VEL_I") params.position.velocity_i_gain(2) = value;
            
            else if (param_name == "MPC_XY_VEL_D") {
                params.position.velocity_d_gain(0) = value;
                params.position.velocity_d_gain(1) = value;
            }
            else if (param_name == "MPC_Z_VEL_D") params.position.velocity_d_gain(2) = value;
            
            // 限制参数
            else if (param_name == "MPC_XY_VEL_MAX") params.position.max_horizontal_velocity = value;
            else if (param_name == "MPC_Z_VEL_MAX_UP") params.position.max_vertical_velocity_up = value;
            else if (param_name == "MPC_Z_VEL_MAX_DN") params.position.max_vertical_velocity_down = value;
            
            else if (param_name == "MPC_THR_MIN") params.position.min_thrust = value;
            else if (param_name == "MPC_THR_MAX") params.position.max_thrust = value;
            else if (param_name == "MPC_THR_HOVER") params.position.hover_thrust = value;
            
            else if (param_name == "MPC_TILTMAX_AIR") params.position.max_tilt_angle = value;
        }
    }
    
    file.close();
    std::cout << "Successfully loaded PX4 parameters from " << param_file_path << std::endl;
    return params;
}