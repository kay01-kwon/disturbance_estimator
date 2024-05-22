#ifndef CONFIG_READ_HPP_
#define CONFIG_READ_HPP_
#include "type_definitions.hpp"
#include <yaml-cpp/yaml.h>
#include <string>
#include <fstream>

using std::string;
using std::cerr;

class Config_Read{

    public:

    Config_Read();

    Config_Read(string& file_name);

    void load_yaml_file();

    void get_Inertial_param_from_yaml(Inertial_param& inertial_param);

    void get_r_offset_from_yaml(mat31_t& r_offset_param);

    void get_Aero_coeff_from_yaml(Aero_coeff& aero_coeff);

    void get_arm_length(double& l);

    private:

    string file_name_;

};

#endif