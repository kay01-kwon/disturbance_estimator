#include "disturbance_estimator.hpp"

DistEst::DistEst()
{
}

DistEst::DistEst(Inertial_param &nominal_param)
{
}

void DistEst::get_est_raw(mat31_t &sigma_est, mat31_t &theta_est)
{
}

void DistEst::get_est_filtered(mat31_t &sigma_est, mat31_t &theta_est)
{
}

void DistEst::system_dynamics(const mat61_t &s, mat61_t &dsdt, double t)
{
}
