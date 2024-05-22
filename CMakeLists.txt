cmake_minimum_required(VERSION 3.0.2)
project(estimator)

find_package(yaml-cpp REQUIRED)

include_directories(
    INCLUDE_DIRS include
    ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(test 
src/estimator_node.cpp
include/tools.cpp
include/config_read.cpp
include/true_model.cpp
)
target_link_libraries(test ${YAML_CPP_LIBRARIES})