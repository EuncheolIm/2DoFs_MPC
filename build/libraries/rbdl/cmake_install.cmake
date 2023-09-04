# Install script for directory: /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/rbdl

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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/rbdl/librbdl.a")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librbdl.a" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librbdl.a")
    execute_process(COMMAND "/usr/bin/ranlib" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librbdl.a")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/rbdl" TYPE FILE FILES
    "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/rbdl/include/rbdl/rbdl_config.h"
    "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/rbdl/include/rbdl/Body.h"
    "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/rbdl/include/rbdl/Constraint.h"
    "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/rbdl/include/rbdl/Constraint_Contact.h"
    "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/rbdl/include/rbdl/Constraint_Loop.h"
    "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/rbdl/include/rbdl/Constraints.h"
    "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/rbdl/include/rbdl/Dynamics.h"
    "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/rbdl/include/rbdl/Joint.h"
    "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/rbdl/include/rbdl/Kinematics.h"
    "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/rbdl/include/rbdl/Logging.h"
    "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/rbdl/include/rbdl/Model.h"
    "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/rbdl/include/rbdl/Quaternion.h"
    "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/rbdl/include/rbdl/SpatialAlgebraOperators.h"
    "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/rbdl/include/rbdl/compileassert.h"
    "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/rbdl/include/rbdl/rbdl.h"
    "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/rbdl/include/rbdl/rbdl_eigenmath.h"
    "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/rbdl/include/rbdl/rbdl_errors.h"
    "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/rbdl/include/rbdl/rbdl_math.h"
    "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/rbdl/include/rbdl/rbdl_mathutils.h"
    "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/rbdl/include/rbdl/rbdl_utils.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/rbdl/rbdl.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/RBDL" TYPE FILE FILES "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/rbdl/RBDLConfig.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/rbdl/addons/urdfreader/cmake_install.cmake")

endif()

