# Install script for directory: /home/kist/euncheol/2DoFs_MPC/libraries/rbdl

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/kist/euncheol/2DoFs_MPC/build/libraries/rbdl/librbdl.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/rbdl" TYPE FILE FILES
    "/home/kist/euncheol/2DoFs_MPC/build/libraries/rbdl/include/rbdl/rbdl_config.h"
    "/home/kist/euncheol/2DoFs_MPC/libraries/rbdl/include/rbdl/Body.h"
    "/home/kist/euncheol/2DoFs_MPC/libraries/rbdl/include/rbdl/Constraint.h"
    "/home/kist/euncheol/2DoFs_MPC/libraries/rbdl/include/rbdl/Constraint_Contact.h"
    "/home/kist/euncheol/2DoFs_MPC/libraries/rbdl/include/rbdl/Constraint_Loop.h"
    "/home/kist/euncheol/2DoFs_MPC/libraries/rbdl/include/rbdl/Constraints.h"
    "/home/kist/euncheol/2DoFs_MPC/libraries/rbdl/include/rbdl/Dynamics.h"
    "/home/kist/euncheol/2DoFs_MPC/libraries/rbdl/include/rbdl/Joint.h"
    "/home/kist/euncheol/2DoFs_MPC/libraries/rbdl/include/rbdl/Kinematics.h"
    "/home/kist/euncheol/2DoFs_MPC/libraries/rbdl/include/rbdl/Logging.h"
    "/home/kist/euncheol/2DoFs_MPC/libraries/rbdl/include/rbdl/Model.h"
    "/home/kist/euncheol/2DoFs_MPC/libraries/rbdl/include/rbdl/Quaternion.h"
    "/home/kist/euncheol/2DoFs_MPC/libraries/rbdl/include/rbdl/SpatialAlgebraOperators.h"
    "/home/kist/euncheol/2DoFs_MPC/libraries/rbdl/include/rbdl/compileassert.h"
    "/home/kist/euncheol/2DoFs_MPC/libraries/rbdl/include/rbdl/rbdl.h"
    "/home/kist/euncheol/2DoFs_MPC/libraries/rbdl/include/rbdl/rbdl_eigenmath.h"
    "/home/kist/euncheol/2DoFs_MPC/libraries/rbdl/include/rbdl/rbdl_errors.h"
    "/home/kist/euncheol/2DoFs_MPC/libraries/rbdl/include/rbdl/rbdl_math.h"
    "/home/kist/euncheol/2DoFs_MPC/libraries/rbdl/include/rbdl/rbdl_mathutils.h"
    "/home/kist/euncheol/2DoFs_MPC/libraries/rbdl/include/rbdl/rbdl_utils.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/kist/euncheol/2DoFs_MPC/build/libraries/rbdl/rbdl.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/RBDL" TYPE FILE FILES "/home/kist/euncheol/2DoFs_MPC/build/libraries/rbdl/RBDLConfig.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/kist/euncheol/2DoFs_MPC/build/libraries/rbdl/addons/urdfreader/cmake_install.cmake")

endif()

