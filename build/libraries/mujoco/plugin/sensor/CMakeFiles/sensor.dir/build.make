# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.27

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /opt/homebrew/Cellar/cmake/3.27.1/bin/cmake

# The command to remove a file.
RM = /opt/homebrew/Cellar/cmake/3.27.1/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/im-euncheol/Desktop/KIST/2DoFs_MPC

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build

# Include any dependencies generated for this target.
include libraries/mujoco/plugin/sensor/CMakeFiles/sensor.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include libraries/mujoco/plugin/sensor/CMakeFiles/sensor.dir/compiler_depend.make

# Include the progress variables for this target.
include libraries/mujoco/plugin/sensor/CMakeFiles/sensor.dir/progress.make

# Include the compile flags for this target's objects.
include libraries/mujoco/plugin/sensor/CMakeFiles/sensor.dir/flags.make

libraries/mujoco/plugin/sensor/CMakeFiles/sensor.dir/sensor.cc.o: libraries/mujoco/plugin/sensor/CMakeFiles/sensor.dir/flags.make
libraries/mujoco/plugin/sensor/CMakeFiles/sensor.dir/sensor.cc.o: /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/plugin/sensor/sensor.cc
libraries/mujoco/plugin/sensor/CMakeFiles/sensor.dir/sensor.cc.o: libraries/mujoco/plugin/sensor/CMakeFiles/sensor.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object libraries/mujoco/plugin/sensor/CMakeFiles/sensor.dir/sensor.cc.o"
	cd /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/plugin/sensor && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT libraries/mujoco/plugin/sensor/CMakeFiles/sensor.dir/sensor.cc.o -MF CMakeFiles/sensor.dir/sensor.cc.o.d -o CMakeFiles/sensor.dir/sensor.cc.o -c /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/plugin/sensor/sensor.cc

libraries/mujoco/plugin/sensor/CMakeFiles/sensor.dir/sensor.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/sensor.dir/sensor.cc.i"
	cd /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/plugin/sensor && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/plugin/sensor/sensor.cc > CMakeFiles/sensor.dir/sensor.cc.i

libraries/mujoco/plugin/sensor/CMakeFiles/sensor.dir/sensor.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/sensor.dir/sensor.cc.s"
	cd /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/plugin/sensor && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/plugin/sensor/sensor.cc -o CMakeFiles/sensor.dir/sensor.cc.s

libraries/mujoco/plugin/sensor/CMakeFiles/sensor.dir/touch_grid.cc.o: libraries/mujoco/plugin/sensor/CMakeFiles/sensor.dir/flags.make
libraries/mujoco/plugin/sensor/CMakeFiles/sensor.dir/touch_grid.cc.o: /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/plugin/sensor/touch_grid.cc
libraries/mujoco/plugin/sensor/CMakeFiles/sensor.dir/touch_grid.cc.o: libraries/mujoco/plugin/sensor/CMakeFiles/sensor.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object libraries/mujoco/plugin/sensor/CMakeFiles/sensor.dir/touch_grid.cc.o"
	cd /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/plugin/sensor && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT libraries/mujoco/plugin/sensor/CMakeFiles/sensor.dir/touch_grid.cc.o -MF CMakeFiles/sensor.dir/touch_grid.cc.o.d -o CMakeFiles/sensor.dir/touch_grid.cc.o -c /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/plugin/sensor/touch_grid.cc

libraries/mujoco/plugin/sensor/CMakeFiles/sensor.dir/touch_grid.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/sensor.dir/touch_grid.cc.i"
	cd /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/plugin/sensor && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/plugin/sensor/touch_grid.cc > CMakeFiles/sensor.dir/touch_grid.cc.i

libraries/mujoco/plugin/sensor/CMakeFiles/sensor.dir/touch_grid.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/sensor.dir/touch_grid.cc.s"
	cd /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/plugin/sensor && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/plugin/sensor/touch_grid.cc -o CMakeFiles/sensor.dir/touch_grid.cc.s

# Object files for target sensor
sensor_OBJECTS = \
"CMakeFiles/sensor.dir/sensor.cc.o" \
"CMakeFiles/sensor.dir/touch_grid.cc.o"

# External object files for target sensor
sensor_EXTERNAL_OBJECTS =

lib/libsensor.dylib: libraries/mujoco/plugin/sensor/CMakeFiles/sensor.dir/sensor.cc.o
lib/libsensor.dylib: libraries/mujoco/plugin/sensor/CMakeFiles/sensor.dir/touch_grid.cc.o
lib/libsensor.dylib: libraries/mujoco/plugin/sensor/CMakeFiles/sensor.dir/build.make
lib/libsensor.dylib: lib/libmujoco.2.3.8.dylib
lib/libsensor.dylib: libraries/mujoco/plugin/sensor/CMakeFiles/sensor.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared library ../../../../lib/libsensor.dylib"
	cd /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/plugin/sensor && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sensor.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
libraries/mujoco/plugin/sensor/CMakeFiles/sensor.dir/build: lib/libsensor.dylib
.PHONY : libraries/mujoco/plugin/sensor/CMakeFiles/sensor.dir/build

libraries/mujoco/plugin/sensor/CMakeFiles/sensor.dir/clean:
	cd /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/plugin/sensor && $(CMAKE_COMMAND) -P CMakeFiles/sensor.dir/cmake_clean.cmake
.PHONY : libraries/mujoco/plugin/sensor/CMakeFiles/sensor.dir/clean

libraries/mujoco/plugin/sensor/CMakeFiles/sensor.dir/depend:
	cd /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/im-euncheol/Desktop/KIST/2DoFs_MPC /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/plugin/sensor /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/plugin/sensor /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/plugin/sensor/CMakeFiles/sensor.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : libraries/mujoco/plugin/sensor/CMakeFiles/sensor.dir/depend

