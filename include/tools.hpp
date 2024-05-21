#ifndef TOOLS_HPP_
#define TOOLS_HPP_
#include "type_definitions.hpp"
#include <iostream>

using std::cout;
using std::endl;

mat31_t grav;

enum QuadModel{model1, model2};

void get_dqdt(quat_t q, mat31_t w, quat_t& dqdt);

void conjugate(quat_t q, quat_t& q_res);

void otimes(quat_t q1, quat_t q2, quat_t& q_res);

void get_Rotm_from_quat(quat_t q, mat33_t& rotm_res);

void vec2skiew(mat31_t vec, mat33_t& skiew_sym_mat);

void quat2quat_vec(quat_t q, mat31_t& q_vec);

void quat2unit_quat(quat_t q, quat_t& unit_q);

double signum(double num);

void gravity_setup();

#endif