#include "low_pass_filter.hpp"
#include "matplotlibcpp.h"
#include <map>
#include <filesystem>
#include <string>

namespace plt = matplotlibcpp;

using std::vector;
using std::chrono::system_clock;
using std::map;
using std::string;


int main()
{

    mat31_t v_noisy, v_lpf;
    int N = 1000;

    double stddev;
    stddev = 0.1;

    Lpf lpf;
    lpf = Lpf(2.0);

    vector<double> v1, v2, v3;
    vector<double> v1_lpf, v2_lpf, v3_lpf, time_vec;
    double time;
    
    map<string, string> label_keywords, 
    font_keywords, legend_keywords;

    string legend_name;

    label_keywords
    .insert(std::pair<std::string, std::string>
    ("linewidth","6"));

    font_keywords
    .insert(std::pair<std::string, std::string>
    ("fontsize","32"));

    legend_keywords
    .insert(std::pair<std::string, std::string>
    ("fontsize","20"));

    legend_keywords
    .insert(std::pair<std::string, std::string>
    ("loc","upper right"));

    for(int i = 0; i < N; i++)
    {
        time = i*0.01;
        long long seedNum = get_seedNum();

        for(int j = 0; j < 3; j++)
            v_noisy(j) = sin(2*M_PI*i/N+M_PI/4.0*j) + noise(stddev, seedNum+j*2);
        
        lpf.apply_input(v_noisy);
        lpf.set_time(time);
        lpf.get_filtered_vector(v_lpf);

        time_vec.push_back(time);
        demux_vec3(v_noisy, v1, v2, v3);
        demux_vec3(v_lpf, v1_lpf, v2_lpf, v3_lpf);

    }

    for(int i = 0; i < 3; i++)
    {
        legend_name = "vector";

    }

    vector<double> x_ticks, y_ticks;
    x_ticks.push_back(0);
    x_ticks.push_back(5);
    x_ticks.push_back(10);
    
    y_ticks.push_back(-1.0);
    y_ticks.push_back(-0.5);
    y_ticks.push_back(0.0);
    y_ticks.push_back(0.5);
    y_ticks.push_back(1.0);

    label_keywords
    .insert(std::pair<std::string, std::string>
    ("label","$v_{x}$"));

    plt::figure_size(3100,1080);
    plt::subplot(1,3,1);
    plt::plot(time_vec, v1, label_keywords);
    label_keywords["label"] = "$v_{x, lpf}$";

    plt::plot(time_vec, v1_lpf, label_keywords);
    plt::xticks(x_ticks,font_keywords);
    plt::yticks(y_ticks,font_keywords);
    plt::legend(legend_keywords);
    plt::title("$v_{x}$ - t",font_keywords);
    plt::xlabel("time (s)",font_keywords);
    plt::ylabel("$v_{x}$",font_keywords);
    plt::grid(true);

    label_keywords["label"] = "$v_{y}$";
    plt::subplot(1,3,2);
    plt::plot(time_vec, v2, label_keywords);
    label_keywords["label"] = "$v_{y, lpf}$";
    plt::plot(time_vec, v2_lpf, label_keywords);
    plt::xticks(x_ticks,font_keywords);
    plt::yticks(y_ticks,font_keywords);
    plt::legend(legend_keywords);
    plt::title("$v_{y}$ - t",font_keywords);
    plt::xlabel("time (s)",font_keywords);
    plt::ylabel("$v_{y}$",font_keywords);
    plt::grid(true);


    label_keywords["label"] = "$v_{z}$";
    plt::subplot(1,3,3);
    plt::plot(time_vec, v3, label_keywords);
    label_keywords["label"] = "$v_{z, lpf}$";
    plt::plot(time_vec, v3_lpf, label_keywords);
    plt::xticks(x_ticks,font_keywords);
    plt::yticks(y_ticks,font_keywords);
    plt::legend(legend_keywords);
    plt::title("$v_{z}$ - t",font_keywords);
    plt::xlabel("time (s)",font_keywords);
    plt::ylabel("$v_{z}$",font_keywords);
    plt::grid(true);

    plt::show();

    return 0;
}