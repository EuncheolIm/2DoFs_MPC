# Install script for directory: /Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco

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

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/plugin/elasticity/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/plugin/sensor/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/src/engine/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/src/user/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/src/xml/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/src/render/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/src/ui/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/model/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/simulate/cmake_install.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "runtime" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/lib/libmujoco.2.3.8.dylib")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmujoco.2.3.8.dylib" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmujoco.2.3.8.dylib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" -x "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmujoco.2.3.8.dylib")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "runtime" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/lib/libmujoco.dylib")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "dev" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mujoco" TYPE FILE FILES
    "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/include/mujoco/mjdata.h"
    "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/include/mujoco/mjexport.h"
    "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/include/mujoco/mjmacro.h"
    "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/include/mujoco/mjmodel.h"
    "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/include/mujoco/mjplugin.h"
    "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/include/mujoco/mjrender.h"
    "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/include/mujoco/mjtnum.h"
    "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/include/mujoco/mjui.h"
    "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/include/mujoco/mjvisualize.h"
    "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/include/mujoco/mjxmacro.h"
    "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/include/mujoco/mujoco.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/mujoco/mujocoTargets.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/mujoco/mujocoTargets.cmake"
         "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/CMakeFiles/Export/0b3b73e44e0b7c8937151edebfc316a5/mujocoTargets.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/mujoco/mujocoTargets-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/mujoco/mujocoTargets.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/mujoco" TYPE FILE FILES "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/CMakeFiles/Export/0b3b73e44e0b7c8937151edebfc316a5/mujocoTargets.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/mujoco" TYPE FILE FILES "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/CMakeFiles/Export/0b3b73e44e0b7c8937151edebfc316a5/mujocoTargets-release.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/mujoco" TYPE FILE FILES
    "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/mujocoConfig.cmake"
    "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/build/libraries/mujoco/mujocoConfigVersion.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mujoco" TYPE DIRECTORY FILES "/Users/im-euncheol/Desktop/KIST/2DoFs_MPC/libraries/mujoco/model" REGEX "/cmakelists\\.txt$" EXCLUDE)
endif()

