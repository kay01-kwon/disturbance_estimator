#ifndef DISTRUBANCE_ESTIMATOR_HPP_
#define DISTRUBANCE_ESTIMATOR_HPP_
#include "convex_fn.hpp"
#include "gamma_proj.hpp"
#include "low_pass_filter.hpp"

class DistEst{

    public:

    DistEst();

    DistEst(Inertial_param& nominal_param);

    void get_est_raw(mat31_t& sigma_est, mat31_t& theta_est);

    void get_est_filtered(mat31_t& sigma_est, mat31_t& theta_est);

    private:
    
    Inertial_param nominal_param_;

    quat_t q_state_, q_hat_;
    mat31_t v_tilde_, w_tilde_;
    mat31_t sigma_hat_, theta_hat_;
    mat31_t sigma_hat_lpf_, theta_hat_lpf_;
    mat31_t dsigma_hat_, dtheta_hat_;

    double curr_time_, prev_time_, dt_;
    
    GammaPrj gamma_prj_obj[2];
    Lpf lpf_obj[2];

};


#endif