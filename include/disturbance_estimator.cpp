#include "disturbance_estimator.hpp"

DistEst::DistEst()
{
    nominal_param_.J.setIdentity();
    nominal_param_.m = 1;

    initial_variables();

    mat31_t bound[2];

    bound[0] << 5, 5, 5;
    bound[1] << 5, 5, 5;

    mat33_t Gamma_Value;

    Gamma_Value.setIdentity();
    Gamma_Value *= 1000.0;

    conv_fn_obj[0] = ConvFn(bound[0],10);
    conv_fn_obj[1] = ConvFn(bound[1],10);
    
    gamma_prj_obj[1] = GammaPrj(Gamma_Value);

    lpf_obj[0] = Lpf(2.0);
    lpf_obj[1] = Lpf(2.0);

}

DistEst::DistEst(inertial_param_t &nominal_param)
:nominal_param_(nominal_param)
{
    mat31_t bound[2];
    initial_variables();

    bound[0] << 5, 5, 5;
    bound[1] << 1, 1, 1;

    mat33_t Gamma_Value1, Gamma_Value2;

    Gamma_Value1.setIdentity();
    Gamma_Value2.setIdentity();

    Gamma_Value1 *= 1000.0;
    Gamma_Value2 *= 0.001;

    conv_fn_obj[0] = ConvFn(bound[0],10);
    conv_fn_obj[1] = ConvFn(bound[1],1);

    gamma_prj_obj[0] = GammaPrj(Gamma_Value1);
    gamma_prj_obj[1] = GammaPrj(Gamma_Value2);

    lpf_obj[0] = Lpf(2.0);
    lpf_obj[1] = Lpf(2.0);
    
}

void DistEst::set_pos(mat31_t p_state, mat31_t p_hat)
{
    p_tilde_ = p_hat - p_state;
}

void DistEst::set_vel(mat31_t v_state, mat31_t v_hat)
{
    v_tilde_ = v_hat - v_state;
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
    mat33_t J_inv, J_inv_transpose;
    mat31_t q_vec;
    double c = 0.0;

    J_inv = nominal_param_.J.inverse();
    J_inv_transpose = J_inv.transpose();

    quat_t q_tilde_unit;

    quat2unit_quat(q_tilde_, q_tilde_unit);

    get_Rotm_from_quat(q_tilde_unit,R);
    quat2quat_vec(q_tilde_, q_vec);
    q_vec = signum(q_tilde_.w()) * q_vec;

    P = nominal_param_.J 
    * R.transpose() 
    * nominal_param_.J.inverse()
    * R;

    P_transpose = P.transpose();

    y_sigma = - v_tilde_;
    y_theta = - P_transpose * w_tilde_
    -c*P_transpose*J_inv_transpose*q_vec;

    double f1, f2;
    mat31_t Df1, Df2;
    
    conv_fn_obj[0].get_fn_value(sigma_hat_, f1, Df1);
    conv_fn_obj[1].get_fn_value(theta_hat_, f2, Df2);

    gamma_prj_obj[0].getProjGamma(sigma_hat_, y_sigma
    , f1, Df1, dsigma_hat_);
    
    gamma_prj_obj[1].getProjGamma(theta_hat_, y_theta
    , f2, Df2, dtheta_hat_);

    solve();

    sigma_est = sigma_hat_;
    theta_est = theta_hat_;

}

void DistEst::get_est_filtered(mat31_t &sigma_hat_lpf, mat31_t &theta_hat_lpf)
{

    lpf_obj[0].apply_input(sigma_hat_);
    lpf_obj[0].set_time(curr_time_);
    lpf_obj[0].get_filtered_vector(sigma_hat_lpf);

    lpf_obj[1].apply_input(theta_hat_);
    lpf_obj[1].set_time(curr_time_);
    lpf_obj[1].get_filtered_vector(theta_hat_lpf);
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
    dt_ = curr_time_ - prev_time_;

    rk4.do_step([this] 
    (const mat61_t& s, mat61_t& dsdt, double t)
    {
        this->DistEst::system_dynamics(s, dsdt, t);
    },
    s_, prev_time_, dt_);

    for(int i =  0; i < 3; i++)
    {
        sigma_hat_(i) = s_(i);
        theta_hat_(i) = s_(i+3);
    }

    prev_time_ = curr_time_;
}

void DistEst::system_dynamics(const mat61_t &s, mat61_t &dsdt, double t)
{
    
    for(int i = 0; i < 3; i++)
    {
        dsdt(i) = dsigma_hat_(i);
        dsdt(i + 3) = dtheta_hat_(i);
    }

}
