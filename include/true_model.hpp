#ifndef TRUE_MODEL_HPP_
#define TRUE_MODEL_HPP_

#include "tools.hpp"
#include <boost/lambda/lambda.hpp>
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>

using boost::numeric::odeint::runge_kutta_dopri5;


class True_model{

    public:

    True_model();

    True_model(QuadModel model,
    Inertial_param& inertial_param,
    Aero_coeff& aero_coeff, double l);

    /**
     * Apply control inputs
    */
    void apply_control_input(mat41_t& rpm);

    /**
     * Apply disturbances
    */
    void apply_disturbance(
        mat31_t& sigma_ext, 
        mat31_t& theta_ext);

    void do_rk_dopri();

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
    Inertial_param inertial_param_;

    /**
     * Control input (Four thrust)
     */
    mat41_t rpm_;

    /**
     * Arm length and moment coefficient info.
    */

    double l_;
    Aero_coeff aero_coeff_;

    /**
     * Thrust to wrench transformation matrix
    */
    mat64_t thrust2wrench_;

    /**
     * force and moment
    */
    mat31_t f_, M_, grav;
    

    /**
     * Declare state and time
    */
    state13_t s_, dsdt_;
    double t_, dt_;

    /**
     * Declare rk dopri5 class
    */
    runge_kutta_dopri5<state13_t> rk_dopri5;

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