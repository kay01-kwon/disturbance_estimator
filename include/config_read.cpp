#include "config_read.hpp"

Config_Read::Config_Read()
{
    file_name_ = "/home/kay/Documents/research/estimator/config/config_param.yaml";
}

Config_Read::Config_Read(string& file_name):
file_name_(file_name)
{
    cout<<"Directory of configuration file name: "<<endl;
    cout<<file_name<<endl;
    load_yaml_file();

}

void Config_Read::load_yaml_file()
{
    try{
        YAML::Node config = YAML::LoadFile(file_name_);
        cout<<"Yaml is file loaded."<<endl;
    }
    catch(const YAML::BadFile& e)
    {
        std::cerr<<e.msg<<endl;
    }
    catch(const YAML::ParserException& e)
    {
        std::cerr<<e.msg<<endl;
    }
}

void Config_Read::get_Inertial_param_from_yaml(Inertial_param& inertial_param)
{

}

void Config_Read::get_r_offset_from_yaml(mat31_t& r_offset_param)
{

}

void Config_Read::get_Aero_coeff_from_yaml(Aero_coeff& aero_coeff)
{

}

void Config_Read::get_arm_length(double& l)
{

}