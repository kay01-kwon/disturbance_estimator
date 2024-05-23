#include "true_model.hpp"
#include "config_read.hpp"
#include "matplotlibcpp.h"
#include "get_state.hpp"

namespace plt = matplotlibcpp;

using std::vector;

int main()
{
    string file_name;
    file_name = "/home/kay/Documents/research/estimator/config/config_param.yaml";
    
    Inertial_param inertial_param;
    Aero_coeff aero_coeff;
    double arm_length;

    /**
     * Configuration read
    */
    Config_Read* config_read_ptr;
    config_read_ptr = new Config_Read(file_name);
    config_read_ptr->get_param(inertial_param, aero_coeff, arm_length);

    delete config_read_ptr;

    print_params(inertial_param, 
    aero_coeff, 
    arm_length);

    /**
     * Load true model
    */

    True_model true_model;
    true_model = True_model(model1, 
    inertial_param, aero_coeff, arm_length);

    vector<double> x, y, z, vx, vy, vz;
    vector<double> qw, qx, qy, qz;
    vector<double> wx, wy, wz;
    vector<double> time_vec;

    mat31_t sigma_ext, theta_ext;
    mat41_t u;

    u.setZero();
    sigma_ext.setZero();
    theta_ext<<0,1,0;

    for(int i = 0; i < 100; i++)
    {
        true_model.apply_control_input(u);
        true_model.apply_disturbance(sigma_ext, theta_ext);
        true_model.do_rk_dopri();

        get_position(&true_model, x, y, z);
        get_velocity(&true_model, vx, vy, vz);
        get_quaternion(&true_model, qw, qx, qy, qz);
        get_angular_velocity(&true_model, wx, wy, wz);
        time_vec.push_back(true_model.get_t());
    }


    plt::figure_size(1960,1080);
    plt::subplot(2,4,1);
    plt::plot(time_vec, qw);
    // // plt::title("$w_{x}$ - t", label_keywords);
    // // plt::xlabel("time (s)",label_keywords);
    // // plt::ylabel("$w_{x}$ (rad/s)",label_keywords);
    plt::grid(true);

    plt::subplot(2,4,2);
    plt::plot(time_vec, qx);
    plt::grid(true);

    plt::subplot(2,4,3);
    plt::plot(time_vec, qy);
    plt::grid(true);

    plt::subplot(2,4,4);
    plt::plot(time_vec, qz);
    plt::grid(true);

    plt::subplot(2,4,5);
    plt::plot(time_vec, wx);
    // // plt::title("$w_{x}$ - t", label_keywords);
    // // plt::xlabel("time (s)",label_keywords);
    // // plt::ylabel("$w_{x}$ (rad/s)",label_keywords);
    plt::grid(true);

    plt::subplot(2,4,6);
    plt::plot(time_vec, wy);
    plt::grid(true);

    plt::subplot(2,4,7);
    plt::plot(time_vec, wz);
    plt::grid(true);

    plt::show();

    return 0;
}
