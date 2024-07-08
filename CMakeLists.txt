cmake_minimum_required(VERSION 3.0.2)
project(estimator)

set(CMAKE_CXX_STANDARD 11)

find_package(yaml-cpp REQUIRED)
find_package(Python3 3.8 EXACT COMPONENTS Interpreter Development)

include_directories(
    INCLUDE_DIRS include
    ${YAML_CPP_INCLUDE_DIRS}
)

# add_executable(estimator_node 
# src/estimator_node.cpp
# include/tools.cpp
# include/config_read.cpp
# include/true_model.cpp
# include/ref_model.cpp
# include/convex_fn.cpp
# include/gamma_proj.cpp
# include/low_pass_filter.cpp
# include/disturbance_estimator.cpp
# )

# target_include_directories(estimator_node PRIVATE 
# ${Python3_INCLUDE_DIRS})

# target_link_libraries(estimator_node 
# ${YAML_CPP_LIBRARIES}
# ${Python3_LIBRARIES} 
# )

add_executable(test 
src/test.cpp
include/tools.cpp
include/config_read.cpp
include/true_model.cpp
include/ref_model.cpp
include/convex_fn.cpp
include/gamma_proj.cpp
include/low_pass_filter.cpp
include/disturbance_estimator.cpp
)

target_include_directories(test PRIVATE 
${Python3_INCLUDE_DIRS})

target_link_libraries(test 
${YAML_CPP_LIBRARIES}
${Python3_LIBRARIES} 
)

# add_executable(test_lpf
# src/test_lpf.cpp
# include/low_pass_filter.cpp
# include/tools.cpp)

# target_include_directories(test_lpf PRIVATE
# ${Python3_INCLUDE_DIRS})

# target_link_libraries(test_lpf
# ${Python3_LIBRARIES}
# )

# add_executable(test_convex_fn_gamma_proj
# src/test_convex_fn_gamma_proj.cpp
# include/gamma_proj.cpp
# include/convex_fn.cpp)