#include "test.hpp"


int main()
{

    file_name = get_current_dir_name();

    file_name = getParentDirectory(file_name);

    file_name = file_name + std::string("/config/config_param.yaml");
    
    /**
     * Configuration read
    */
    config_read_ptr = new Config_Read(file_name);
    config_read_ptr->get_param(inertial_param, aero_coeff, arm_length);

    delete config_read_ptr;

    print_params(inertial_param, aero_coeff, arm_length);


    /**
     * Load true model
    */
    model = model1;
    true_model = True_model(model, 
    inertial_param, aero_coeff, arm_length);
    ref_model = Ref_Model(inertial_param);
    estimator = DistEst(inertial_param);


    map<string, string> label_keywords, 
    font_keywords, legend_keywords;

    string legend_name;

    label_keywords
    .insert(std::pair<string, string>
    ("linewidth","6"));

    font_keywords
    .insert(std::pair<string, string>
    ("fontsize","32"));

    legend_keywords
    .insert(std::pair<string, string>
    ("fontsize","20"));

    legend_keywords
    .insert(std::pair<string, string>
    ("loc","upper right"));

    u.setZero();
    sigma_ext << 1, -2, 1.5;
    theta_ext << 0, 0, 0;

    mat31_t theta_est, sigma_est;
    mat31_t theta_est_lpf, sigma_est_lpf;
    theta_est.setZero();
    sigma_est.setZero();
    theta_est_lpf.setZero();
    sigma_est_lpf.setZero();
    

    for(int i = 0; i < 3000; i++)
    {
        mat31_t p_state, v_state, w_state;
        quat_t q_state;
        true_model.apply_control_input(u);
        true_model.apply_disturbance(sigma_ext, theta_ext);
        true_model.do_rk4();

        true_model.get_pos_from_state(p_state);
        true_model.get_vel_from_state(v_state);
        true_model.get_quat_from_state(q_state);
        true_model.get_angular_vel_from_state(w_state);

        demux_vec3(p_state, x, y, z);
        demux_vec3(v_state, vx, vy, vz);
        demux_quat(q_state, qw, qx, qy, qz);
        demux_vec3(w_state, wx, wy, wz);
        
        time_vec.push_back(true_model.get_t());

        mat31_t u1, u2;

        if(true_model.get_t() > 3)
            sigma_ext << -1, 2, 0.5;
        theta_ext << sin(2*true_model.get_t()), 
        -sin(1.2*true_model.get_t()),
        2*cos(4*true_model.get_t());
        // theta_ext.setZero();

        u1.setZero();
        u2.setZero();

        ref_model.set_quat_angular_vel(q_state, w_state);
        ref_model.set_est_disturbance(sigma_est, theta_est);
        ref_model.apply_input(u1, u2);
        ref_model.set_time(true_model.get_t());
        ref_model.solve();

        mat31_t p_ref, v_ref, w_ref;
        quat_t q_ref;

        ref_model.get_pos_from_ref_model(p_ref);
        ref_model.get_vel_from_ref_model(v_ref);
        ref_model.get_quat_from_ref_model(q_ref);
        ref_model.get_angular_vel_from_ref_model(w_ref);

        estimator.set_vel(v_state, v_ref);
        estimator.set_angular_vel(w_state, w_ref,
        q_state, q_ref);
        estimator.set_time(true_model.get_t());

        estimator.get_est_raw(sigma_est, theta_est);
        estimator.get_est_filtered(sigma_est_lpf, theta_est_lpf);

        demux_vec3(v_ref, vx_ref, vy_ref, vz_ref);
        demux_quat(q_ref, qw_ref, qx_ref, qy_ref, qz_ref);
        demux_vec3(w_ref, wx_ref, wy_ref, wz_ref);

        demux_vec3(sigma_est, sigma_x, sigma_y, sigma_z);
        demux_vec3(sigma_est_lpf, sigma_x_lpf, sigma_y_lpf, sigma_z_lpf);
        demux_vec3(sigma_ext, sigma_x_g, sigma_y_g, sigma_z_g);

        demux_vec3(theta_est, theta_x, theta_y, theta_z);
        demux_vec3(theta_est_lpf, theta_x_lpf, theta_y_lpf, theta_z_lpf);
        demux_vec3(theta_ext, theta_x_g, theta_y_g, theta_z_g);
    }

    x_ticks.push_back(0);
    x_ticks.push_back(5);
    x_ticks.push_back(10);
    x_ticks.push_back(15);
    x_ticks.push_back(20);

    y_ticks.push_back(-4);
    y_ticks.push_back(-2);
    y_ticks.push_back(0.0);
    y_ticks.push_back(2.0);
    y_ticks.push_back(4);

    plt::figure_size(3500,2000);

    plt::subplot(2,3,1);
    plt::plot(time_vec, sigma_x, label_keywords);
    plt::plot(time_vec, sigma_x_lpf, label_keywords);
    plt::plot(time_vec, sigma_x_g, label_keywords);
    plt::title("$σ_{x}$ - t", font_keywords);
    plt::xlabel("time (s)",font_keywords);
    plt::ylabel("$σ_{x}$",font_keywords);
    plt::xticks(x_ticks,font_keywords);
    plt::yticks(y_ticks,font_keywords);
    plt::grid(true);

    plt::subplot(2,3,2);
    plt::plot(time_vec, sigma_y, label_keywords);
    plt::plot(time_vec, sigma_y_lpf, label_keywords);
    plt::plot(time_vec, sigma_y_g, label_keywords);
    
    plt::title("$σ_{y}$ - t", font_keywords);
    plt::xlabel("time (s)",font_keywords);
    plt::ylabel("$σ_{y}$",font_keywords);
    plt::xticks(x_ticks,font_keywords);
    plt::yticks(y_ticks,font_keywords);
    plt::grid(true);

    plt::subplot(2,3,3);
    plt::plot(time_vec, sigma_z, label_keywords);
    plt::plot(time_vec, sigma_z_lpf, label_keywords);
    plt::plot(time_vec, sigma_z_g, label_keywords);
    plt::title("$σ_{z}$ - t", font_keywords);
    plt::xlabel("time (s)",font_keywords);
    plt::ylabel("$σ_{z}$",font_keywords);
    plt::xticks(x_ticks,font_keywords);
    plt::yticks(y_ticks,font_keywords);
    plt::grid(true);


    plt::subplot(2,3,4);
    plt::plot(time_vec, theta_x, label_keywords);
    plt::plot(time_vec, theta_x_lpf, label_keywords);
    plt::plot(time_vec, theta_x_g, label_keywords);
    plt::title("$θ_{x}$ - t", font_keywords);
    plt::xlabel("time (s)",font_keywords);
    plt::ylabel("$θ_{x}$",font_keywords);
    plt::xticks(x_ticks,font_keywords);
    plt::yticks(y_ticks,font_keywords);
    plt::grid(true);

    plt::subplot(2,3,5);
    plt::plot(time_vec, theta_y, label_keywords);
    plt::plot(time_vec, theta_y_lpf, label_keywords);
    plt::plot(time_vec, theta_y_g, label_keywords);
    plt::title("$θ_{y}$ - t", font_keywords);
    plt::xlabel("time (s)",font_keywords);
    plt::ylabel("$θ_{y}$",font_keywords);
    plt::xticks(x_ticks,font_keywords);
    plt::yticks(y_ticks,font_keywords);
    plt::grid(true);

    plt::subplot(2,3,6);
    plt::plot(time_vec, theta_z, label_keywords);
    plt::plot(time_vec, theta_z_lpf, label_keywords);
    plt::plot(time_vec, theta_z_g, label_keywords);
    plt::title("$θ_{z}$ - t", font_keywords);
    plt::xlabel("time (s)",font_keywords);
    plt::ylabel("$θ_{z}$",font_keywords);
    plt::xticks(x_ticks,font_keywords);
    plt::yticks(y_ticks,font_keywords);
    plt::grid(true);

    plt::figure_size(3500,2000);

    plt::subplot(2,3,1);
    plt::plot(time_vec, vx_ref, label_keywords); 
    plt::plot(time_vec, vx, label_keywords);
    plt::title("$v_{x}$ - t", font_keywords);
    plt::xlabel("time (s)",font_keywords);
    plt::ylabel("$v_{x}$",font_keywords);
    // plt::xticks(x_ticks,font_keywords);
    // plt::yticks(y_ticks,font_keywords);
    plt::grid(true);

    plt::subplot(2,3,2);
    plt::plot(time_vec, vy_ref, label_keywords);
    plt::plot(time_vec, vy, label_keywords);    
    plt::title("$v_{y}$ - t", font_keywords);
    plt::xlabel("time (s)",font_keywords);
    plt::ylabel("$v_{y}$",font_keywords);
    // plt::xticks(x_ticks,font_keywords);
    // plt::yticks(y_ticks,font_keywords);
    plt::grid(true);

    plt::subplot(2,3,3);
    plt::plot(time_vec, vz_ref, label_keywords);
    plt::plot(time_vec, vz, label_keywords);
    plt::title("$v_{z}$ - t", font_keywords);
    plt::xlabel("time (s)",font_keywords);
    plt::ylabel("$v_{z}$",font_keywords);
    // plt::xticks(x_ticks,font_keywords);
    // plt::yticks(y_ticks,font_keywords);
    plt::grid(true);

    plt::subplot(2,3,4);
    plt::plot(time_vec, wx_ref, label_keywords); 
    plt::plot(time_vec, wx, label_keywords);
    plt::title("$w_{x}$ - t", font_keywords);
    plt::xlabel("time (s)",font_keywords);
    plt::ylabel("$w_{x}$",font_keywords);
    // plt::xticks(x_ticks,font_keywords);
    // plt::yticks(y_ticks,font_keywords);
    plt::grid(true);

    plt::subplot(2,3,5);
    plt::plot(time_vec, wy_ref, label_keywords);
    plt::plot(time_vec, wy, label_keywords);    
    plt::title("$w_{y}$ - t", font_keywords);
    plt::xlabel("time (s)",font_keywords);
    plt::ylabel("$w_{y}$",font_keywords);
    // plt::xticks(x_ticks,font_keywords);
    // plt::yticks(y_ticks,font_keywords);
    plt::grid(true);

    plt::subplot(2,3,6);
    plt::plot(time_vec, wz_ref, label_keywords);
    plt::plot(time_vec, wz, label_keywords);
    plt::title("$w_{z}$ - t", font_keywords);
    plt::xlabel("time (s)",font_keywords);
    plt::ylabel("$w_{z}$",font_keywords);
    // plt::xticks(x_ticks,font_keywords);
    // plt::yticks(y_ticks,font_keywords);
    plt::grid(true);

    plt::figure_size(3500,2000);
    plt::subplot(1,4,1);
    plt::plot(time_vec, qw_ref, label_keywords);
    plt::plot(time_vec, qw, label_keywords);
    plt::title("$q_{w}$ - t", font_keywords);
    plt::xlabel("time (s)",font_keywords);
    plt::ylabel("$q_{w}$",font_keywords);
    plt::xticks(x_ticks,font_keywords);
    plt::yticks(y_ticks,font_keywords);
    plt::grid(true);

    plt::subplot(1,4,2);
    plt::plot(time_vec, qx_ref, label_keywords);
    plt::plot(time_vec, qx, label_keywords);
    plt::title("$q_{x}$ - t", font_keywords);
    plt::xlabel("time (s)",font_keywords);
    plt::ylabel("$q_{x}$",font_keywords);
    plt::xticks(x_ticks,font_keywords);
    plt::yticks(y_ticks,font_keywords);
    plt::grid(true);

    plt::subplot(1,4,3);
    plt::plot(time_vec, qy_ref, label_keywords);
    plt::plot(time_vec, qy, label_keywords);
    plt::title("$q_{y}$ - t", font_keywords);
    plt::xlabel("time (s)",font_keywords);
    plt::ylabel("$q_{y}$",font_keywords);
    plt::xticks(x_ticks,font_keywords);
    plt::yticks(y_ticks,font_keywords);
    plt::grid(true);

    plt::subplot(1,4,4);
    plt::plot(time_vec, qz_ref, label_keywords);
    plt::plot(time_vec, qz, label_keywords);
    plt::title("$q_{z}$ - t", font_keywords);
    plt::xlabel("time (s)",font_keywords);
    plt::ylabel("$q_{z}$",font_keywords);
    plt::xticks(x_ticks,font_keywords);
    plt::yticks(y_ticks,font_keywords);
    plt::grid(true);

    plt::show();

    return 0;
}
