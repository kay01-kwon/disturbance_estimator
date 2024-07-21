#ifndef TEST_HPP_
#define TEST_HPP_
#include "true_model.hpp"
#include "config_read.hpp"
#include "disturbance_estimator.hpp"
#include "ref_model.hpp"
#include "matplotlibcpp.h"
#include <map>
#include <unistd.h>
#include <limits.h>
#include <string>

namespace plt = matplotlibcpp;

using std::vector;
using std::map;

string file_name;

inertial_param_t inertial_param;
aero_coeff_t aero_coeff;
double arm_length;

Config_Read* config_read_ptr;

True_model true_model;
QuadModel model;

Ref_Model ref_model;
DistEst estimator;

vector<double> x, y, z, vx, vy, vz;
vector<double> vx_ref, vy_ref, vz_ref;
vector<double> qw, qx, qy, qz;
vector<double> qw_ref, qx_ref, qy_ref, qz_ref;
vector<double> wx, wy, wz;
vector<double> wx_ref, wy_ref, wz_ref;
vector<double> time_vec;

vector<double> sigma_x, sigma_y, sigma_z;
vector<double> sigma_x_lpf, sigma_y_lpf, sigma_z_lpf;
vector<double> sigma_x_g, sigma_y_g, sigma_z_g;

vector<double> theta_x, theta_y, theta_z;
vector<double> theta_x_lpf, theta_y_lpf, theta_z_lpf;
vector<double> theta_x_g, theta_y_g, theta_z_g;

mat31_t sigma_ext, theta_ext;
mat41_t u;

vector<double> x_ticks, y_ticks;

#endif