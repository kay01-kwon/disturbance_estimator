#include "ref_model.hpp"

Ref_Model::Ref_Model()
{
    nominal_param_.J.setIdentity();
    nominal_param_.m = 1;
    
    u_hat_.setZero();
    mu_hat_.setZero();

    s_hat_.setZero();
    s_hat_(6) = 1.0;
    dsdt_hat_.setZero();

    grav.setZero();
    grav(2) = -9.81;

    curr_time_ = prev_time_ = dt_ = 0;
}

Ref_Model::Ref_Model(Inertial_param &nominal_param)
:nominal_param_(nominal_param)
{
    u_hat_.setZero();
    mu_hat_.setZero();

    s_hat_.setZero();
    s_hat_(6) = 1.0;
    dsdt_hat_.setZero();

    grav.setZero();
    grav(2) = -9.81;

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

void Ref_Model::solve()
{
    dt_ = curr_time_ - prev_time_;
    rk4.do_step(
        std::bind(
        &Ref_Model::ref_dynamics,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3
        ),
        s_hat_, prev_time_, dt_
    );
    prev_time_ = curr_time_;
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

void Ref_Model::ref_dynamics(const state13_t &s, state13_t &dsdt, double t)
{
    mat31_t p,v,dpdt,dvdt;
    quat_t q, q_unit, dqdt;
    mat31_t w, dwdt;
    mat33_t R, w_skiew;

    for(int i = 0; i < 3; i++)
    {
        p(i) = s(i);
        v(i) = s(i+3);
    }

    q.w() = s(6);
    q.x() = s(7);
    q.y() = s(8);
    q.z() = s(9);
    
    quat2unit_quat(q, q_unit);

    for(int i = 0; i < 3; i++)
    {
        w(i) = s(i+10);
    }

    dpdt = v;
    dvdt = (1/nominal_param_.m)*u_hat_ + grav;

    get_dqdt(q_unit, w, dqdt);
    dwdt = nominal_param_.J.inverse()*mu_hat_;

    for(int i = 0; i < 3; i++)
    {
        dsdt(i) = dpdt(i);
        dsdt(i+3) = dvdt(i);
    }

    dsdt(6) = dqdt.w();
    dsdt(7) = dqdt.x();
    dsdt(8) = dqdt.y();
    dsdt(9) = dqdt.z();

    for(int i = 0; i < 3; i++)
    {
        dsdt(i+10) = dwdt(i);
    }

}