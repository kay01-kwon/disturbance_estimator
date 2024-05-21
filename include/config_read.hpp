#ifndef CONFIG_READ_HPP_
#define CONFIG_READ_HPP_
#include "tools.hpp"
#include <yaml-cpp/yaml.h>
#include <string>
#include <fstream>

using std::string;
using std::cerr;

class Config_Read{

    public:

    Config_Read();

    Config_Read(string& file_name);

    Inertial_param& get_Inertial_param_from_yaml();

    mat31_t& get_r_offset_from_yaml();

    Aero_coeff& get_Aero_coeff_from_yaml();

    double get_arm_length();

    ~Config_Read();

    private:

    YAML::Node config;

    string file_name_;

};

#endif