#ifndef TYPE_DEFINITIONS_HPP_
#define TYPE_DEFINITIONS_HPP_

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using Eigen::Matrix;
using Eigen::Quaternion;

using std::cout;
using std::endl;

typedef Matrix<double,3,1> mat31_t;
typedef Matrix<double,3,3> mat33_t;

typedef Matrix<double,4,4> mat44_t;
typedef Matrix<double,4,1> mat41_t;

typedef Quaternion<double> quat_t;

#endif