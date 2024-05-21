#include "true_model.hpp"

True_model::True_model()
{
    gravity_setup();

    cout<<"Default setup"<<endl;

    model_config(model1);

    r_offset_ << 0, 0, 0;

}

True_model::True_model(
QuadModel model, 
double m, mat33_t J, mat31_t r_offset,
double lift_coeff, double moment_coeff, double l):
m_(m), J_(J), r_offset_(r_offset),
lift_coeff_(lift_coeff), moment_coeff_(moment_coeff),
l_(l)
{
    gravity_setup();

    model_config(model);
}

void True_model::apply_control_input(mat41_t rpm)
{
    mat61_t wrench;
    mat41_t T;
    mat33_t r_offset_skiew;
    mat31_t collective_thrust;

    for(int i = 0; i < 4; i++)
    {
        T(i) = lift_coeff_ * pow(rpm(i),2);
    }

    wrench = thrust2wrench_ * T;
    collective_thrust << 0, 0, wrench(2);
    vec2skiew(r_offset_, r_offset_skiew);

    for(int i = 0; i < 3; i++)
    {
        f_(i) = wrench(i);
        M_(i) = wrench(i+3);
    }

    M_ += r_offset_skiew * collective_thrust;

}

void True_model::apply_disturbance(
    mat31_t sigma_ext,
    mat31_t theta_ext)
{
    f_ += sigma_ext;
    M_ += theta_ext;
}

state13_t& True_model::get_s()
{
    return s_;
}

state13_t& True_model::get_dsdt()
{
    return dsdt_;
}

double& True_model::get_t()
{
    return t_;
}

mat31_t& True_model::get_pos_from_state()
{
    mat31_t p;
    
    for(int i = 0; i < 3; i++)
        p(i) = s_(i);

    return p;
}

mat31_t& True_model::get_vel_from_state()
{
    mat31_t v;

    for(int i = 0; i < 3; i++)
        v(i) = s_(i+3);
    
    return v;
}

quat_t& True_model::get_quat_from_state()
{
    quat_t q, q_unit;

    q.w() = s_(6);
    q.x() = s_(7);
    q.y() = s_(8);
    q.z() = s_(9);

    quat2unit_quat(q,q_unit);
    
    return q;
}

mat31_t& True_model::get_angular_vel_from_state()
{
    mat31_t w;
    w(0) = s_(10);
    w(1) = s_(11);
    w(2) = s_(12);
    return w;
}

void True_model::system_dynamics(
    const state13_t& s,
    state13_t& dsdt,
    double t
)
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

    vec2skiew(w, w_skiew);

    get_Rotm_from_quat(q, R);

    dpdt = v;
    dvdt = (1/m_)*R*f_ + grav;

    get_dqdt(q, w, dqdt);
    dwdt = J_.inverse()*(M_ - w_skiew*(J_*w));

    for(int i = 0; i < 3; i++)
    {
        dsdt(i) = p(i);
        dsdt(i+3) = v(i);
    }

    dsdt(6) = q.w();
    dsdt(7) = q.x();
    dsdt(8) = q.y();
    dsdt(9) = q.z();

    for(int i = 0; i < 3; i++)
    {
        dsdt(i+10) = dwdt(i);
    }

}

void True_model::do_rk_dopri()
{
    rk_dopri5.do_step(
        std::bind(
            &True_model::system_dynamics,
            &(*this),
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3
        ),
        s_,
        t_,
        dt_
    );
    t_ += dt_;
}

void True_model::model_config(QuadModel model)
{

    double sinPI4, l1, Cm;
    sinPI4 = sin(M_PI/4.0);
    l1 = l_*sinPI4;
    Cm = moment_coeff_/lift_coeff_;

    if(model == model1)
    {
        cout<<"Quad model: x (model1)"<<endl;
        // Put thrust2wrench_ matrix 
        // corresponding to x model
        thrust2wrench_<<    0, 0, 0, 0,
                            0, 0, 0, 0,
                            1, 1, 1, 1,
                            l1, -l1, -l1, l1,
                            -l1, -l1, l1, l1,
                            Cm, -Cm, Cm, -Cm;

    }
    else if (model == model2)
    {
        cout<<"Quad model: + (model2)"<<endl;
        // Put thrust2wrench_ matrix
        // according to + model
        thrust2wrench_<<    0, 0, 0, 0,
                            0, 0, 0, 0,
                            1, 1, 1, 1,
                            0, -l_, 0, l_,
                            -l_, 0, l_, 0,
                            Cm, -Cm, Cm, -Cm;
                            
    }
    else
    {
        cout<<"Invalid model"<<endl;
        cout<<"Designate model1 or model2"<<endl;
        cout<<"Shutdown the program"<<endl;
        exit(1);
    }
}