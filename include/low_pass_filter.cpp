#include "low_pass_filter.hpp"

Lpf::Lpf()
{
    tau_ = 2.0;
    initialize_variables();
}

Lpf::Lpf(double tau):tau_(tau)
{
    initialize_variables();
}

void Lpf::initialize_variables()
{
    curr_v_.setZero();
    curr_time_ = prev_time_ = dt_ = 0;
}

void Lpf::apply_input(mat31_t v_in)
{
    v_in_ = v_in;
}

void Lpf::set_time(double t)
{
    curr_time_ = t;
}

void Lpf::get_filtered_vector(mat31_t& v_out)
{
    do_rk_dopri();
    v_out = v_out_;
}

void Lpf::system_dynamics(const mat31_t& v,
mat31_t& dvdt,
double t)
{
    
}

void Lpf::do_rk_dopri()
{

}