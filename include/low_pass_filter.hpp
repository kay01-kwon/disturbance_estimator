#ifndef LOW_PASS_FILTER_HPP_
#define LOW_PASS_FILTER_HPP_
#include "type_definitions.hpp"
#include <boost/lambda/lambda.hpp>
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>

using boost::numeric::odeint::runge_kutta_dopri5;

class Lpf{

    public:

    Lpf();

    Lpf(double tau);

    void apply_input(mat31_t v_in);

    void set_time(double t);

    void get_filtered_vector(mat31_t& v_out);

    private:

    // vector to be filtered
    mat31_t curr_v_, v_in_;

    // time constant
    double tau_;

    // Time info
    double curr_time, prev_time, dt;

    /**
     * Declare rk dopri5 class
    */
    runge_kutta_dopri5<state13_t> rk_dopri5;

    void system_dynamics(
        const mat31_t& v,
        mat31_t& dvdt,
        double t);

    void do_rk_dopri();
};


#endif