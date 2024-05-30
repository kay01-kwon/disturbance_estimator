#ifndef REF_MODEL_HPP_
#define REF_MODEL_HPP_
#include "tools.hpp"
#include <boost/lambda/lambda.hpp>
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>

using boost::numeric::odeint::runge_kutta4;

class Ref_Model{

    public:

    Ref_Model();

    Ref_Model(Inertial_param& nominal_param);

    /**
     * Apply inputs
     * u_comp = u - C(s)*sigma_hat
     * mu_comp = mu - C(s)*theta_hat  
    */
    void apply_input(mat31_t u_comp, mat31_t mu_comp);

    void set_quat_angular_vel(quat_t q_state, mat31_t w_state);

    void set_est_disturbance(mat31_t sigma_est,
    mat31_t theta_est);

    void set_time(double t);

    /**
     * Step integration of rungee kutta 4th order.
    */
    void solve();

    /**
     * Get position, velocity, quaternion 
     * and angular velocity, respectively.
    */
    void get_pos_from_ref_model(mat31_t& p_ref);

    void get_vel_from_ref_model(mat31_t& v_ref);

    void get_quat_from_ref_model(quat_t& q_ref);

    void get_angular_vel_from_ref_model(mat31_t& w_ref);

    private:

    Inertial_param nominal_param_;

    mat31_t u_hat_, mu_hat_;

    mat31_t theta_hat_, sigma_hat_;

    mat31_t grav;

    state13_t s_hat_, dsdt_hat_;
    mat31_t p_hat_, v_hat_, w_hat_, w_state_;
    mat31_t w_tilde_;

    quat_t q_hat_, q_tilde_;
    double curr_time_, prev_time_, dt_;

    //runge kutta 4 class Declaration
    runge_kutta4<state13_t> rk4;

    /**
     * Convert mu_comp to mu_hat.
    */
    void mu_comp2mu_hat(mat31_t mu_comp, mat31_t& mu_hat);

    /**
     * Dynamics of reference model
    */

    void ref_dynamics(
        const state13_t& s,
        state13_t& dsdt,
        double t
    );

};

#endif