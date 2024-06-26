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

void Ref_Model::apply_input(mat31_t u_comp, mat31_t mu_comp)
{
    u_hat_ = u_comp;
    mu_comp2mu_hat(mu_comp, mu_hat_);
}

void Ref_Model::set_quat_angular_vel(quat_t q_state,
mat31_t w_state)
{
    // Get the current state of quaternion
    // and then compute the tilde of quaternion.
    quat_t q_state_conj;

    conjugate(q_state, q_state_conj);
    otimes(q_state_conj, q_hat_, q_tilde_);

    // Get the current angular velocity
    w_state_ = w_state;

}

void Ref_Model::set_pos_vel(mat31_t p_state, mat31_t v_state)
{
    double k_p, k_d;
    mat31_t p_tilde, v_tilde;
    k_p = 5;
    k_d = 2;

    p_tilde = p_hat_ - p_state;
    v_tilde = v_hat_ - v_state;

    u_hat_ -= (k_p*p_tilde + k_d*v_tilde);
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
    
    for(int i = 0; i < 3; i++)
    {
        p_hat_(i) = s_hat_(i);
        v_hat_(i) = s_hat_(i+3);
        w_hat_(i) = s_hat_(i+10);
    }

    q_hat_.w() = s_hat_(6);
    q_hat_.x() = s_hat_(7);
    q_hat_.y() = s_hat_(8);
    q_hat_.z() = s_hat_(9);


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
    mat33_t C, R, skiew_sym;
    mat31_t q_vec;
    double k_q, k_w;

    k_q = 5.0;
    k_w = 2.0;

    // Get rotation matrix from q_tilde_
    get_Rotm_from_quat(q_tilde_, R);
    quat2quat_vec(q_tilde_,q_vec);

    q_vec = signum(q_tilde_.w())*q_vec;

    // Get the error of angular velocity
    w_tilde_ = w_hat_ - R.transpose()*w_state_;

    C = nominal_param_.J 
    * R.transpose() 
    * nominal_param_.J.inverse();

    vec2skiew(w_tilde_, skiew_sym);

    mu_hat = C
    *(mu_comp + R*theta_hat_)
    - nominal_param_.J
    *skiew_sym
    *R.transpose()*w_state_
    -k_q*q_vec - k_w*w_tilde_;

}

void Ref_Model::ref_dynamics(const state13_t &s, state13_t &dsdt, double t)
{
    mat31_t p,v,dpdt,dvdt;
    quat_t q, q_unit, dqdt;
    mat31_t w, dwdt;

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
    dvdt = (1/nominal_param_.m)*(u_hat_ + sigma_hat_) + grav;

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