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
include _deps/qhull-build/CMakeFiles/user_eg3.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include _deps/qhull-build/CMakeFiles/user_eg3.dir/compiler_depend.make

# Include the progress variables for this target.
include _deps/qhull-build/CMakeFiles/user_eg3.dir/progress.make

# Include the compile flags for this target's objects.
include _deps/qhull-build/CMakeFiles/user_eg3.dir/flags.make

_deps/qhull-build/CMakeFiles/user_eg3.dir/src/user_eg3/user_eg3_r.cpp.o: _deps/qhull-build/CMakeFiles/user_eg3.dir/flags.make
_deps/qhull-build/CMakeFiles/user_eg3.dir/src/user_eg3/user_eg3_r.cpp.o: _deps/qhull-src/src/user_eg3/user_eg3_r.cpp
_deps/qhull-build/CMakeFiles/user_eg3.dir/src/user_eg3/user_eg3_r.cpp.o: _deps/qhull-build/CMakeFiles/user_eg3.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object _deps/qhull-build/CMakeFiles/user_eg3.dir/src/user_eg3/user_eg3_r.cpp.o"
	cd /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/_deps/qhull-build && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT _deps/qhull-build/CMakeFiles/user_eg3.dir/src/user_eg3/user_eg3_r.cpp.o -MF CMakeFiles/user_eg3.dir/src/user_eg3/user_eg3_r.cpp.o.d -o CMakeFiles/user_eg3.dir/src/user_eg3/user_eg3_r.cpp.o -c /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/_deps/qhull-src/src/user_eg3/user_eg3_r.cpp

_deps/qhull-build/CMakeFiles/user_eg3.dir/src/user_eg3/user_eg3_r.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/user_eg3.dir/src/user_eg3/user_eg3_r.cpp.i"
	cd /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/_deps/qhull-build && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/_deps/qhull-src/src/user_eg3/user_eg3_r.cpp > CMakeFiles/user_eg3.dir/src/user_eg3/user_eg3_r.cpp.i

_deps/qhull-build/CMakeFiles/user_eg3.dir/src/user_eg3/user_eg3_r.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/user_eg3.dir/src/user_eg3/user_eg3_r.cpp.s"
	cd /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/_deps/qhull-build && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/_deps/qhull-src/src/user_eg3/user_eg3_r.cpp -o CMakeFiles/user_eg3.dir/src/user_eg3/user_eg3_r.cpp.s

# Object files for target user_eg3
user_eg3_OBJECTS = \
"CMakeFiles/user_eg3.dir/src/user_eg3/user_eg3_r.cpp.o"

# External object files for target user_eg3
user_eg3_EXTERNAL_OBJECTS =

bin/user_eg3: _deps/qhull-build/CMakeFiles/user_eg3.dir/src/user_eg3/user_eg3_r.cpp.o
bin/user_eg3: _deps/qhull-build/CMakeFiles/user_eg3.dir/build.make
bin/user_eg3: lib/libqhullcpp.a
bin/user_eg3: lib/libqhullstatic_r.a
bin/user_eg3: _deps/qhull-build/CMakeFiles/user_eg3.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/user_eg3"
	cd /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/_deps/qhull-build && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/user_eg3.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
_deps/qhull-build/CMakeFiles/user_eg3.dir/build: bin/user_eg3
.PHONY : _deps/qhull-build/CMakeFiles/user_eg3.dir/build

_deps/qhull-build/CMakeFiles/user_eg3.dir/clean:
	cd /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/_deps/qhull-build && $(CMAKE_COMMAND) -P CMakeFiles/user_eg3.dir/cmake_clean.cmake
.PHONY : _deps/qhull-build/CMakeFiles/user_eg3.dir/clean

_deps/qhull-build/CMakeFiles/user_eg3.dir/depend:
	cd /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/im-euncheol/Desktop/KIST/2DoFs_MPC /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/_deps/qhull-src /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/_deps/qhull-build /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/_deps/qhull-build/CMakeFiles/user_eg3.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : _deps/qhull-build/CMakeFiles/user_eg3.dir/depend

