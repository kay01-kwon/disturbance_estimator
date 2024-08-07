#ifndef TRUE_MODEL_HPP_
#define TRUE_MODEL_HPP_

#include "tools.hpp"
#include <boost/lambda/lambda.hpp>
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>

using boost::numeric::odeint::runge_kutta4;

class True_model{

    public:

    True_model() = default;

    True_model(const QuadModel& model,
    const inertial_param_t& inertial_param,
    const aero_coeff_t& aero_coeff, 
    const double& l, 
    const double& dt);

    /**
     * Apply control inputs
    */
    void set_control_input(const mat41_t& rpm);

    /**
     * Apply disturbances
    */
    void set_disturbance(
        const mat31_t& sigma_ext, 
        const mat31_t& theta_ext);

    void do_rk4();

    /**
     * Get state, time derivative of that, 
     * and time, respectively
    */
    state13_t& get_s();
    state13_t& get_dsdt();
    double& get_t();

    void get_pos_from_state(mat31_t& p);
    void get_vel_from_state(mat31_t& v);

    void get_quat_from_state(quat_t& q);
    void get_angular_vel_from_state(mat31_t& w);

    private:

    // Inertial parameter
    inertial_param_t inertial_param_;

    /**
     * Control input (Four thrust)
     */
    mat41_t rpm_;

    /**
     * Arm length and moment coefficient info.
    */

    double l_;
    aero_coeff_t aero_coeff_;

    /**
     * Thrust to wrench transformation matrix
    */
    mat64_t thrust2wrench_;

    /**
     * force and moment
    */
    mat31_t f_, M_, grav, sigma_ext_;
    

    /**
     * Declare state and time
    */
    state13_t s_, dsdt_;
    double t_, dt_;

    /**
     * Declare rk4 class
    */

    runge_kutta4<state13_t> rk4;
    

    /**
     * Quadrotor Model configuration
     * Choose one among 'x' or '+'
    */

    void model_config(QuadModel model);

    /**
     * System dynamics to perform rungee kuta dopri5
    */
    void system_dynamics(
        const state13_t& s, 
        state13_t& dsdt, 
        double t);

};


#endif