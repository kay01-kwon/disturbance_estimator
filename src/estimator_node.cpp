#include "true_model.hpp"
#include "config_read.hpp"

int main()
{
    string file_name;
    file_name = "/home/kay/Documents/research/estimator/config/config_param.yaml";
    Config_Read config_read(file_name);

    return 0;
}