# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/kay/Documents/research/estimator

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kay/Documents/research/estimator/build

# Include any dependencies generated for this target.
include CMakeFiles/test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test.dir/flags.make

CMakeFiles/test.dir/src/estimator_node.cpp.o: CMakeFiles/test.dir/flags.make
CMakeFiles/test.dir/src/estimator_node.cpp.o: ../src/estimator_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kay/Documents/research/estimator/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test.dir/src/estimator_node.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test.dir/src/estimator_node.cpp.o -c /home/kay/Documents/research/estimator/src/estimator_node.cpp

CMakeFiles/test.dir/src/estimator_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test.dir/src/estimator_node.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kay/Documents/research/estimator/src/estimator_node.cpp > CMakeFiles/test.dir/src/estimator_node.cpp.i

CMakeFiles/test.dir/src/estimator_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test.dir/src/estimator_node.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kay/Documents/research/estimator/src/estimator_node.cpp -o CMakeFiles/test.dir/src/estimator_node.cpp.s

CMakeFiles/test.dir/include/tools.cpp.o: CMakeFiles/test.dir/flags.make
CMakeFiles/test.dir/include/tools.cpp.o: ../include/tools.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kay/Documents/research/estimator/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/test.dir/include/tools.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test.dir/include/tools.cpp.o -c /home/kay/Documents/research/estimator/include/tools.cpp

CMakeFiles/test.dir/include/tools.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test.dir/include/tools.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kay/Documents/research/estimator/include/tools.cpp > CMakeFiles/test.dir/include/tools.cpp.i

CMakeFiles/test.dir/include/tools.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test.dir/include/tools.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kay/Documents/research/estimator/include/tools.cpp -o CMakeFiles/test.dir/include/tools.cpp.s

CMakeFiles/test.dir/include/config_read.cpp.o: CMakeFiles/test.dir/flags.make
CMakeFiles/test.dir/include/config_read.cpp.o: ../include/config_read.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kay/Documents/research/estimator/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/test.dir/include/config_read.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test.dir/include/config_read.cpp.o -c /home/kay/Documents/research/estimator/include/config_read.cpp

CMakeFiles/test.dir/include/config_read.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test.dir/include/config_read.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kay/Documents/research/estimator/include/config_read.cpp > CMakeFiles/test.dir/include/config_read.cpp.i

CMakeFiles/test.dir/include/config_read.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test.dir/include/config_read.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kay/Documents/research/estimator/include/config_read.cpp -o CMakeFiles/test.dir/include/config_read.cpp.s

CMakeFiles/test.dir/include/true_model.cpp.o: CMakeFiles/test.dir/flags.make
CMakeFiles/test.dir/include/true_model.cpp.o: ../include/true_model.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kay/Documents/research/estimator/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/test.dir/include/true_model.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test.dir/include/true_model.cpp.o -c /home/kay/Documents/research/estimator/include/true_model.cpp

CMakeFiles/test.dir/include/true_model.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test.dir/include/true_model.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kay/Documents/research/estimator/include/true_model.cpp > CMakeFiles/test.dir/include/true_model.cpp.i

CMakeFiles/test.dir/include/true_model.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test.dir/include/true_model.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kay/Documents/research/estimator/include/true_model.cpp -o CMakeFiles/test.dir/include/true_model.cpp.s

CMakeFiles/test.dir/include/get_state.cpp.o: CMakeFiles/test.dir/flags.make
CMakeFiles/test.dir/include/get_state.cpp.o: ../include/get_state.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kay/Documents/research/estimator/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/test.dir/include/get_state.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test.dir/include/get_state.cpp.o -c /home/kay/Documents/research/estimator/include/get_state.cpp

CMakeFiles/test.dir/include/get_state.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test.dir/include/get_state.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kay/Documents/research/estimator/include/get_state.cpp > CMakeFiles/test.dir/include/get_state.cpp.i

CMakeFiles/test.dir/include/get_state.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test.dir/include/get_state.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kay/Documents/research/estimator/include/get_state.cpp -o CMakeFiles/test.dir/include/get_state.cpp.s

# Object files for target test
test_OBJECTS = \
"CMakeFiles/test.dir/src/estimator_node.cpp.o" \
"CMakeFiles/test.dir/include/tools.cpp.o" \
"CMakeFiles/test.dir/include/config_read.cpp.o" \
"CMakeFiles/test.dir/include/true_model.cpp.o" \
"CMakeFiles/test.dir/include/get_state.cpp.o"

# External object files for target test
test_EXTERNAL_OBJECTS =

test: CMakeFiles/test.dir/src/estimator_node.cpp.o
test: CMakeFiles/test.dir/include/tools.cpp.o
test: CMakeFiles/test.dir/include/config_read.cpp.o
test: CMakeFiles/test.dir/include/true_model.cpp.o
test: CMakeFiles/test.dir/include/get_state.cpp.o
test: CMakeFiles/test.dir/build.make
test: /usr/lib/x86_64-linux-gnu/libyaml-cpp.so.0.6.2
test: /usr/lib/x86_64-linux-gnu/libpython3.8.so
test: CMakeFiles/test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kay/Documents/research/estimator/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test.dir/build: test

.PHONY : CMakeFiles/test.dir/build

CMakeFiles/test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test.dir/clean

CMakeFiles/test.dir/depend:
	cd /home/kay/Documents/research/estimator/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kay/Documents/research/estimator /home/kay/Documents/research/estimator /home/kay/Documents/research/estimator/build /home/kay/Documents/research/estimator/build /home/kay/Documents/research/estimator/build/CMakeFiles/test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test.dir/depend

