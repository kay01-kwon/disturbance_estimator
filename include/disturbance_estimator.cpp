#include "disturbance_estimator.hpp"

DistEst::DistEst()
{
    nominal_param_.J.setIdentity();
    nominal_param_.m = 1;

    initial_variables();

}

DistEst::DistEst(Inertial_param &nominal_param)
:nominal_param_(nominal_param)
{
    mat31_t bound[2];
    initial_variables();

    bound[0] << 5, 5, 5;
    bound[1] << 3, 3, 3;

    conv_fn_obj[0] = ConvFn(bound[0],1);
    conv_fn_obj[1] = ConvFn(bound[1],1);
    

    lpf_obj[0] = Lpf(2.0);
    lpf_obj[1] = Lpf(2.0);
    
}

void DistEst::set_vel(mat31_t v_state, mat31_t v_hat)
{
    v_tilde_ = v_state - v_hat;
}

void DistEst::set_angular_vel(mat31_t w_state, mat31_t w_hat, 
quat_t q_state, quat_t q_hat)
{
    quat_t q_conj;
    mat33_t R;
    conjugate(q_state, q_conj);
    otimes(q_conj, q_hat, q_tilde_);
    get_Rotm_from_quat(q_tilde_,R);
    w_tilde_ = w_hat - R.transpose()*w_state;
}

void DistEst::get_est_raw(mat31_t &sigma_est, mat31_t &theta_est)
{
}

void DistEst::get_est_filtered(mat31_t &sigma_est, mat31_t &theta_est)
{
}

void DistEst::initial_variables()
{
    sigma_hat_.setZero();
    theta_hat_.setZero();

    sigma_hat_lpf_.setZero();
    theta_hat_lpf_.setZero();
    
    dsigma_hat_.setZero();
    dtheta_hat_.setZero();

    curr_time_ = prev_time_ = dt_ = 0;
}

void DistEst::system_dynamics(const mat61_t &s, mat61_t &dsdt, double t)
{
}
