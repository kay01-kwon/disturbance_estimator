#include "config_read.hpp"

Config_Read::Config_Read()
{
    file_name_ = "/home/kay/Documents/research/estimator/config/config_param.yaml";
}

Config_Read::Config_Read(string& file_name):
file_name_(file_name)
{

}

Inertial_param& Config_Read::get_Inertial_param_from_yaml()
{

}

mat31_t& Config_Read::get_r_offset_from_yaml()
{

}
