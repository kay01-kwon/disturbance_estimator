#ifndef TYPE_DEFINITIONS_HPP_
#define TYPE_DEFINITIONS_HPP_

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using Eigen::Matrix;
using Eigen::Quaternion;

using std::cout;
using std::endl;

enum ODE_method{rk4_method, rk45_method, dorpri_method};

typedef Matrix<double,3,1> mat31_t;
typedef Matrix<double,3,3> mat33_t;

typedef Matrix<double,4,4> mat44_t;
typedef Matrix<double,4,1> mat41_t;

typedef Matrix<double,6,1> mat61_t;
typedef Matrix<double,6,4> mat64_t;

/**
 * state6_t:
 * position (x, y, z)
 * velocity (vx, vy, vz)
 * 
 * state7_t:
 * quaternion (qw, qx, qy, qz)
 * angular velocity (wx, wy, wz)
 * 
 * state13_t:
 * position (x, y, z)
 * velocity (vx, vy, vz)
 * quaternion (qw, qx, qy, qz)
 * angular velocity (wx, wy, wz)
*/
typedef Matrix<double,6,1> state6_t;
typedef Matrix<double,7,1> state7_t;
typedef Matrix<double,13,1> state13_t;

typedef Quaternion<double> quat_t;

typedef struct Aero_coeff_{
    double moment_coeff;
    double lift_coeff;
} aero_coeff_t;

typedef struct Inertial_param_{
    double m;
    mat33_t J;
    mat31_t r_offset;
} inertial_param_t;

#endif