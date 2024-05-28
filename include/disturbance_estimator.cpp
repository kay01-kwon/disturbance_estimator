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

void DistEst::set_time(double t)
{
    curr_time_ = t;
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
    mat31_t y_sigma, y_theta;
    mat33_t R, P, P_transpose;

    get_Rotm_from_quat(q_tilde_,R);

    P = nominal_param_.J * 
    R.transpose() * 
    nominal_param_.J.inverse();

    P_transpose = P.transpose();

    y_sigma = - v_tilde_;
    y_theta = - P_transpose * w_tilde_;

    double f1, f2;
    mat31_t Df1, Df2;
    
    conv_fn_obj[0].get_fn_value(sigma_hat_, f1, Df1);
    conv_fn_obj[1].get_fn_value(theta_hat_, f2, Df2);

    gamma_prj_obj[0].getProjGamma(sigma_hat_, y_sigma
    , f1, Df1, dsigma_hat_);
    
    gamma_prj_obj[1].getProjGamma(theta_hat_, y_theta
    , f2, Df2, dtheta_hat_);

}

void DistEst::get_est_filtered(mat31_t &sigma_est, mat31_t &theta_est)
{

    lpf_obj[0].apply_input(sigma_hat_);
    lpf_obj[0].set_time(curr_time_);
    lpf_obj[0].get_filtered_vector(sigma_hat_lpf_);

    mat33_t R;
    get_Rotm_from_quat(q_tilde_,R);
    
    lpf_obj[1].apply_input(R*theta_hat_);
    lpf_obj[1].set_time(curr_time_);
    lpf_obj[1].get_filtered_vector(theta_hat_lpf_);
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

void DistEst::solve()
{

}

void DistEst::system_dynamics(const mat61_t &s, mat61_t &dsdt, double t)
{
}
