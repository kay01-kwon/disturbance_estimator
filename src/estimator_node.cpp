#include "true_model.hpp"
#include "config_read.hpp"

int main()
{
    string file_name;
    file_name = "/home/kay/Documents/research/estimator/config/config_param.yaml";
    
    Inertial_param inertial_param;
    Aero_coeff aero_coeff;
    double arm_length;

    Config_Read* config_read_ptr;
    config_read_ptr = new Config_Read(file_name);
    config_read_ptr->get_param(inertial_param, aero_coeff, arm_length);

    delete config_read_ptr;

    print_params(inertial_param, 
    aero_coeff, 
    arm_length);

    True_model* true_model_ptr;
    true_model_ptr = new True_model(model1, 
    inertial_param, aero_coeff, arm_length);

    mat31_t p, v, w;
    double time;
    quat_t q;

    for(int i = 0; i < 100; i++)
    {
        true_model_ptr->do_rk_dopri();
        true_model_ptr->get_pos_from_state(p);
        time = true_model_ptr->get_t();
        cout<<p<<endl;
        cout<<"\n";
        cout<<time<<endl;
        cout<<"\n";
    }


    delete true_model_ptr;

    return 0;
}