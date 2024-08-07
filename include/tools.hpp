#ifndef TOOLS_HPP_
#define TOOLS_HPP_
#include "type_definitions.hpp"
#include <iostream>
#include <random>
#include <chrono>
#include <ctime>
#include <unistd.h>
#include <limits.h>
#include <string>

using std::cout;
using std::endl;
using std::mt19937;
using std::chrono::system_clock;
using std::chrono::duration_cast;
using std::vector;
using std::string;

enum QuadModel{model1, model2};

void get_dqdt(quat_t q, mat31_t w, quat_t& dqdt);

void conjugate(quat_t q, quat_t& q_res);

void otimes(quat_t q1, quat_t q2, quat_t& q_res);

void get_Rotm_from_quat(quat_t q, mat33_t& rotm_res);

void vec2skiew(mat31_t vec, mat33_t& skiew_sym_mat);

void quat2quat_vec(quat_t q, mat31_t& q_vec);

void quat2unit_quat(quat_t q, quat_t& unit_q);

double signum(double num);

void print_params(inertial_param_t& param1, aero_coeff_t& param2, double& param3);

double noise(double stddev, long long seedNum);

long long get_seedNum();

void demux_vec3(mat31_t v, vector<double>&x, vector<double>&y, vector<double>&z);

void demux_quat(quat_t q, vector<double>&qw, vector<double>&qx, vector<double>&qy, vector<double>&qz);

std::string getCurrentWorkingDirectory();

std::string getParentDirectory(const std::string& path);

#endif