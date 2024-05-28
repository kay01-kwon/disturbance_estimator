#ifndef ESTIMATOR_NODE_HPP_
#define ESTIMATOR_NODE_HPP_
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

vector<double> x, y, z, vx, vy, vz;
vector<double> qw, qx, qy, qz;
vector<double> wx, wy, wz;
vector<double> time_vec;

mat31_t sigma_ext, theta_ext;
mat41_t u;

vector<double> x_ticks, y_ticks;

#endif