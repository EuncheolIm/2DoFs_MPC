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
include libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/compiler_depend.make

# Include the progress variables for this target.
include libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/progress.make

# Include the compile flags for this target's objects.
include libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/flags.make

libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/glfw_adapter.cc.o: libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/flags.make
libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/glfw_adapter.cc.o: /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/simulate/glfw_adapter.cc
libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/glfw_adapter.cc.o: libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/glfw_adapter.cc.o"
	cd /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/simulate && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/glfw_adapter.cc.o -MF CMakeFiles/platform_ui_adapter.dir/glfw_adapter.cc.o.d -o CMakeFiles/platform_ui_adapter.dir/glfw_adapter.cc.o -c /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/simulate/glfw_adapter.cc

libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/glfw_adapter.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/platform_ui_adapter.dir/glfw_adapter.cc.i"
	cd /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/simulate && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/simulate/glfw_adapter.cc > CMakeFiles/platform_ui_adapter.dir/glfw_adapter.cc.i

libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/glfw_adapter.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/platform_ui_adapter.dir/glfw_adapter.cc.s"
	cd /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/simulate && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/simulate/glfw_adapter.cc -o CMakeFiles/platform_ui_adapter.dir/glfw_adapter.cc.s

libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/glfw_dispatch.cc.o: libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/flags.make
libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/glfw_dispatch.cc.o: /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/simulate/glfw_dispatch.cc
libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/glfw_dispatch.cc.o: libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/glfw_dispatch.cc.o"
	cd /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/simulate && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/glfw_dispatch.cc.o -MF CMakeFiles/platform_ui_adapter.dir/glfw_dispatch.cc.o.d -o CMakeFiles/platform_ui_adapter.dir/glfw_dispatch.cc.o -c /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/simulate/glfw_dispatch.cc

libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/glfw_dispatch.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/platform_ui_adapter.dir/glfw_dispatch.cc.i"
	cd /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/simulate && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/simulate/glfw_dispatch.cc > CMakeFiles/platform_ui_adapter.dir/glfw_dispatch.cc.i

libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/glfw_dispatch.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/platform_ui_adapter.dir/glfw_dispatch.cc.s"
	cd /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/simulate && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/simulate/glfw_dispatch.cc -o CMakeFiles/platform_ui_adapter.dir/glfw_dispatch.cc.s

libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/platform_ui_adapter.cc.o: libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/flags.make
libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/platform_ui_adapter.cc.o: /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/simulate/platform_ui_adapter.cc
libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/platform_ui_adapter.cc.o: libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/platform_ui_adapter.cc.o"
	cd /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/simulate && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/platform_ui_adapter.cc.o -MF CMakeFiles/platform_ui_adapter.dir/platform_ui_adapter.cc.o.d -o CMakeFiles/platform_ui_adapter.dir/platform_ui_adapter.cc.o -c /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/simulate/platform_ui_adapter.cc

libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/platform_ui_adapter.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/platform_ui_adapter.dir/platform_ui_adapter.cc.i"
	cd /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/simulate && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/simulate/platform_ui_adapter.cc > CMakeFiles/platform_ui_adapter.dir/platform_ui_adapter.cc.i

libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/platform_ui_adapter.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/platform_ui_adapter.dir/platform_ui_adapter.cc.s"
	cd /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/simulate && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/simulate/platform_ui_adapter.cc -o CMakeFiles/platform_ui_adapter.dir/platform_ui_adapter.cc.s

libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/glfw_corevideo.mm.o: libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/flags.make
libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/glfw_corevideo.mm.o: /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/simulate/glfw_corevideo.mm
libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/glfw_corevideo.mm.o: libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building OBJCXX object libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/glfw_corevideo.mm.o"
	cd /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/simulate && /Library/Developer/CommandLineTools/usr/bin/clang++ $(OBJCXX_DEFINES) $(OBJCXX_INCLUDES) -x objective-c++ $(OBJCXX_FLAGS) -MD -MT libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/glfw_corevideo.mm.o -MF CMakeFiles/platform_ui_adapter.dir/glfw_corevideo.mm.o.d -o CMakeFiles/platform_ui_adapter.dir/glfw_corevideo.mm.o -c /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/simulate/glfw_corevideo.mm

libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/glfw_corevideo.mm.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing OBJCXX source to CMakeFiles/platform_ui_adapter.dir/glfw_corevideo.mm.i"
	cd /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/simulate && /Library/Developer/CommandLineTools/usr/bin/clang++ $(OBJCXX_DEFINES) $(OBJCXX_INCLUDES) $(OBJCXX_FLAGS) -E /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/simulate/glfw_corevideo.mm > CMakeFiles/platform_ui_adapter.dir/glfw_corevideo.mm.i

libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/glfw_corevideo.mm.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling OBJCXX source to assembly CMakeFiles/platform_ui_adapter.dir/glfw_corevideo.mm.s"
	cd /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/simulate && /Library/Developer/CommandLineTools/usr/bin/clang++ $(OBJCXX_DEFINES) $(OBJCXX_INCLUDES) $(OBJCXX_FLAGS) -S /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/simulate/glfw_corevideo.mm -o CMakeFiles/platform_ui_adapter.dir/glfw_corevideo.mm.s

platform_ui_adapter: libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/glfw_adapter.cc.o
platform_ui_adapter: libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/glfw_dispatch.cc.o
platform_ui_adapter: libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/platform_ui_adapter.cc.o
platform_ui_adapter: libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/glfw_corevideo.mm.o
platform_ui_adapter: libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/build.make
.PHONY : platform_ui_adapter

# Rule to build all files generated by this target.
libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/build: platform_ui_adapter
.PHONY : libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/build

libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/clean:
	cd /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/simulate && $(CMAKE_COMMAND) -P CMakeFiles/platform_ui_adapter.dir/cmake_clean.cmake
.PHONY : libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/clean

libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/depend:
	cd /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/im-euncheol/Desktop/KIST/2DoFs_MPC /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/simulate /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/simulate /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : libraries/mujoco/simulate/CMakeFiles/platform_ui_adapter.dir/depend
