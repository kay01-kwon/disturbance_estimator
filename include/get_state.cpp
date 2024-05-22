#include "get_state.hpp"

void get_position(True_model* true_model_ptr, 
vector<double>&x, vector<double>&y, vector<double>&z)
{
    mat31_t p;
    true_model_ptr->get_pos_from_state(p);

    x.push_back(p(0));
    y.push_back(p(1));
    z.push_back(p(2));
}

void get_velocity(True_model* true_model_ptr, 
vector<double>&vx, vector<double>&vy, vector<double>&vz)
{
    mat31_t v;

    true_model_ptr->get_vel_from_state(v);

    vx.push_back(v(0));
    vy.push_back(v(1));
    vz.push_back(v(2));
}

void get_quaternion(True_model* true_model_ptr, 
vector<double>&qw, vector<double>&qx, 
vector<double>&qy, vector<double>&qz)
{
    quat_t q;

    true_model_ptr->get_quat_from_state(q);

    qw.push_back(q.w());
    qx.push_back(q.x());
    qy.push_back(q.y());
    qz.push_back(q.z());
}

void get_angular_velocity(True_model* true_model_ptr, 
vector<double>&wx, vector<double>&wy, vector<double>&wz)
{
    mat31_t w;

    true_model_ptr->get_angular_vel_from_state(w);

    wx.push_back(w(0));
    wy.push_back(w(1));
    wz.push_back(w(2));
}