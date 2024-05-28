#include "ref_model.hpp"

Ref_Model::Ref_Model()
{
    nominal_param_.J.setIdentity();
    nominal_param_.m = 1;
    
    u_hat_.setZero();
    M_hat_.setZero();

    s_hat_.setZero();
    s_hat_(6) = 1.0;
    dsdt_hat_.setZero();

    curr_time_ = prev_time_ = dt_ = 0;
}

Ref_Model::Ref_Model(Inertial_param &nominal_param)
:nominal_param_(nominal_param)
{
    u_hat_.setZero();
    M_hat_.setZero();

    s_hat_.setZero();
    s_hat_(6) = 1.0;
    dsdt_hat_.setZero();

    curr_time_ = prev_time_ = dt_ = 0;
}

void Ref_Model::apply_input(mat31_t &u_comp, mat31_t &mu_comp)
{
    u_hat_ = u_comp;

}

void Ref_Model::set_quat_angular_vel(quat_t &q_state,
mat31_t& w_state)
{
    // Get the current state of quaternion
    // and then compute the tilde of quaternion.
    quat_t q_state_conj;

    conjugate(q_state, q_state_conj);
    otimes(q_state_conj, q_hat_, q_tilde_);

    // Get the current angular velocity
    w_state_ = w_state;

}

void Ref_Model::set_est_disturbance(mat31_t sigma_est, mat31_t theta_est)
{
    sigma_hat_ = sigma_est;
    theta_hat_ = theta_est;
}

void Ref_Model::set_time(double t)
{
    curr_time_ = t;
}

void Ref_Model::get_pos_from_ref_model(mat31_t &p_ref)
{
    p_ref = p_hat_;
}

void Ref_Model::get_vel_from_ref_model(mat31_t &v_ref)
{
    v_ref = v_hat_;
}

void Ref_Model::get_quat_from_ref_model(quat_t &q_ref)
{
    q_ref = q_hat_;
}

void Ref_Model::get_angular_vel_from_ref_model(mat31_t &w_ref)
{
    w_ref = w_hat_;
}

void Ref_Model::mu_comp2mu_hat(mat31_t mu_comp, mat31_t &mu_hat)
{
    mat33_t P, R, skiew_sym;

    // Get rotation matrix from q_tilde_
    get_Rotm_from_quat(q_tilde_, R);

    // Get the error of angular velocity
    w_tilde_ = w_hat_ - R.transpose()*w_state_;

    P = nominal_param_.J * 
    R.transpose() * nominal_param_.J.inverse();

    vec2skiew(w_tilde_, skiew_sym);

    mu_hat = P
    *(mu_comp + R*theta_hat_)
    - nominal_param_.J
    *skiew_sym
    *R.transpose()*w_state_;

}

void Ref_Model::mu_hat2M_hat(mat31_t mu_hat, mat31_t &M_hat)
{
    mat33_t skiew_sym;
    vec2skiew(w_hat_, skiew_sym);
    M_hat = mu_hat + skiew_sym*(nominal_param_.J*w_hat_);
}

void Ref_Model::ref_dynamics(const state13_t &s, state13_t &dsdt, double t)
{
}

void Ref_Model::solve()
{
}
