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

    cout<<"Inertial Parameter setup"<<endl;

    cout<<"Mass: ";
    cout<<inertial_param.m<<endl;

    cout<<"MOI: ";
    cout<<inertial_param.J<<endl;

    cout<<"Aerodynamics coefficients setup "<<endl;
    cout<<"Lift coefficient: ";
    cout<<aero_coeff.lift_coeff<<endl;

    cout<<"Moment coefficient:";
    cout<<aero_coeff.moment_coeff<<endl;

    cout<<"Arm length: "<<arm_length<<endl;

    delete config_read_ptr;

    return 0;
}