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

Inertial_param inertial_param;
Aero_coeff aero_coeff;
double arm_length;

Config_Read* config_read_ptr;

True_model true_model;
QuadModel model;

Ref_Model ref_model;
DistEst estimator;

vector<double> x, y, z, vx, vy, vz;
vector<double> qw, qx, qy, qz;
vector<double> wx, wy, wz;
vector<double> time_vec;

vector<double> sigma_x, sigma_y, sigma_z;
vector<double> sigma_x_lpf, sigma_y_lpf, sigma_z_lpf;

vector<double> theta_x, theta_y, theta_z;
vector<double> theta_x_lpf, theta_y_lpf, theta_z_lpf;

mat31_t sigma_ext, theta_ext;
mat41_t u;

vector<double> x_ticks, y_ticks;

#endif