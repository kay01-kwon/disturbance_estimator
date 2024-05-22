#ifndef GET_STATE_HPP_
#define GET_STATE_HPP_
#include "tools.hpp"
#include "true_model.hpp"

using std::vector;

void get_position(True_model* true_model_ptr, 
vector<double>&x, vector<double>&y, vector<double>&z);

void get_velocity(True_model* true_model_ptr, 
vector<double>&vx, vector<double>&vy, vector<double>&vz);

void get_quaternion(True_model* true_model_ptr, 
vector<double>&qw, vector<double>&qx, 
vector<double>&qy, vector<double>&qz);

void get_angular_velocity(True_model* true_model_ptr, 
vector<double>&wx, vector<double>&wy, vector<double>&wz);


#endif