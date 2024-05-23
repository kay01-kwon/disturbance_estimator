#include "true_model.hpp"
#include "config_read.hpp"
#include "matplotlibcpp.h"
#include "get_state.hpp"
#include <map>
#include <filesystem>
#include <string>


namespace plt = matplotlibcpp;

using std::vector;
using std::map;
using std::string;

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

    map<string, string> label_keywords, 
    font_keywords, legend_keywords;

    string legend_name;

    label_keywords
    .insert(std::pair<string, string>
    ("linewidth","6"));

    font_keywords
    .insert(std::pair<string, string>
    ("fontsize","32"));

    legend_keywords
    .insert(std::pair<string, string>
    ("fontsize","20"));

    legend_keywords
    .insert(std::pair<string, string>
    ("loc","upper right"));

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

    vector<double> x_ticks, y_ticks;
    x_ticks.push_back(0);
    x_ticks.push_back(0.5);
    x_ticks.push_back(1.0);
    
    y_ticks.push_back(-1.0);
    y_ticks.push_back(-0.5);
    y_ticks.push_back(0.0);
    y_ticks.push_back(0.5);
    y_ticks.push_back(1.0);


    plt::figure_size(3500,2000);
    plt::subplot(2,4,1);
    plt::plot(time_vec, qw, label_keywords);
    plt::title("$q_{w}$ - t", font_keywords);
    plt::xlabel("time (s)",font_keywords);
    plt::ylabel("$q_{w}$",font_keywords);
    plt::xticks(x_ticks,font_keywords);
    plt::yticks(y_ticks,font_keywords);
    plt::grid(true);

    plt::subplot(2,4,2);
    plt::plot(time_vec, qx, label_keywords);
    plt::title("$q_{x}$ - t", font_keywords);
    plt::xlabel("time (s)",font_keywords);
    plt::ylabel("$q_{x}$",font_keywords);
    plt::xticks(x_ticks,font_keywords);
    plt::yticks(y_ticks,font_keywords);
    plt::grid(true);

    plt::subplot(2,4,3);
    plt::plot(time_vec, qy, label_keywords);
    plt::title("$q_{y}$ - t", font_keywords);
    plt::xlabel("time (s)",font_keywords);
    plt::ylabel("$q_{y}$",font_keywords);
    plt::xticks(x_ticks,font_keywords);
    plt::yticks(y_ticks,font_keywords);
    plt::grid(true);

    plt::subplot(2,4,4);
    plt::plot(time_vec, qz, label_keywords);
    plt::title("$q_{z}$ - t", font_keywords);
    plt::xlabel("time (s)",font_keywords);
    plt::ylabel("$q_{z}$",font_keywords);
    plt::xticks(x_ticks,font_keywords);
    plt::yticks(y_ticks,font_keywords);
    plt::grid(true);

    plt::subplot(2,4,5);
    plt::plot(time_vec, wx, label_keywords);
    plt::title("$ω_{x}$ - t", font_keywords);
    plt::xlabel("time (s)",font_keywords);
    plt::ylabel("$ω_{x}$",font_keywords);
    plt::xticks(x_ticks,font_keywords);
    plt::yticks(y_ticks,font_keywords);
    plt::grid(true);

    vector<double> y_ticks_new;


    y_ticks_new.push_back(0.0);
    y_ticks_new.push_back(25);
    y_ticks_new.push_back(50);
    y_ticks_new.push_back(75);
    y_ticks_new.push_back(100.0);

    for(auto& iter: y_ticks)
        cout<<iter<<endl;

    plt::subplot(2,4,6);
    plt::plot(time_vec, wy,label_keywords);
    plt::title("$ω_{y}$ - t", font_keywords);
    plt::xlabel("time (s)",font_keywords);
    plt::ylabel("$ω_{y}$",font_keywords);
    plt::xticks(x_ticks,font_keywords);
    plt::yticks(y_ticks_new,font_keywords);
    plt::grid(true);

    plt::subplot(2,4,7);
    plt::plot(time_vec, wz,label_keywords);
    plt::title("$ω_{z}$ - t", font_keywords);
    plt::xlabel("time (s)",font_keywords);
    plt::ylabel("$ω_{z}$",font_keywords);
    plt::xticks(x_ticks,font_keywords);
    plt::yticks(y_ticks,font_keywords);
    plt::grid(true);

    plt::show();

    return 0;
}
