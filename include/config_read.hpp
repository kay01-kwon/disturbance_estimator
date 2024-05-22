#ifndef CONFIG_READ_HPP_
#define CONFIG_READ_HPP_
#include "type_definitions.hpp"
#include <yaml-cpp/yaml.h>
#include <string>
#include <fstream>

using std::string;
using std::cerr;
using YAML::Node;

class Config_Read{

    public:

    Config_Read();

    Config_Read(string& file_name);

    void get_param(Inertial_param& inertial_param,
    Aero_coeff& aero_coeff, double& arm_length);

    private:

    string file_name_;

    Inertial_param inertial_param_;

    Aero_coeff aero_coeff_;

    double l_;

    void load_yaml_file();

    void get_Inertial_param_from_yaml(Node& config);

    void get_Aero_coeff_from_yaml(Node& config);

    void get_arm_length_from_yaml(Node& config);

};

#endif