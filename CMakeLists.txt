cmake_minimum_required(VERSION 3.15)
project(sm)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_CXX_FLAGS} -Wl,-no-as-needed")
#fing eigen library
# find_package (Eigen3 3.3.7 REQUIRED)
# message(STATUS "Eigen3 : " ${EIGEN3_INCLUDE_DIR}/Eigen)
# message(STATUS "current : " ${CMAKE_CURRENT_SOURCE_DIR})

option(MUJOCO_BUILD_EXAMPLES "Build samples for MuJoCo" OFF)
option(MUJOCO_BUILD_SIMULATE "Build simulate library for MuJoCo" ON)
option(MUJOCO_BUILD_TESTS "Build tests for MuJoCo" OFF)
option(MUJOCO_TEST_PYTHON_UTIL "Build and test utility libraries for Python bindings" ON)
option(QPOASES_BUILD_EXAMPLES "Build examples." OFF)

add_subdirectory(${PROJECT_SOURCE_DIR}/libraries/mujoco)                             # Add subdirectory
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/libraries/mujoco/sample/cmake") # CMake Path append
add_subdirectory(${PROJECT_SOURCE_DIR}/libraries/qpOASES)                            # Add subdirectory

# Check if we are building as standalone project.
set(SAMPLE_STANDALONE OFF)
set(_INSTALL_SAMPLES ON)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(SAMPLE_STANDALONE ON)
  # If standalone, do not install the samples.
  set(_INSTALL_SAMPLES OFF)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(SAMPLE_STANDALONE)
  include(SampleOptions)
else()
  enforce_mujoco_macosx_min_version()
endif()
include(SampleDependencies)

set(MUJOCO_SAMPLE_COMPILE_OPTIONS "${AVX_COMPILE_OPTIONS}" "${EXTRA_COMPILE_OPTIONS}")
set(MUJOCO_SAMPLE_LINK_OPTIONS "${EXTRA_LINK_OPTIONS}")

if(MUJOCO_HARDEN)
  if(WIN32)
    set(MUJOCO_SAMPLE_LINK_OPTIONS "${MUJOCO_SAMPLE_LINK_OPTIONS}" -Wl,/DYNAMICBASE)
  else()
    set(MUJOCO_SAMPLE_COMPILE_OPTIONS "${MUJOCO_SAMPLE_COMPILE_OPTIONS}" -fPIE)
    if(APPLE)
      set(MUJOCO_SAMPLE_LINK_OPTIONS "${MUJOCO_SAMPLE_LINK_OPTIONS}" -Wl,-pie)
    else()
      set(MUJOCO_SAMPLE_LINK_OPTIONS "${MUJOCO_SAMPLE_LINK_OPTIONS}" -pie)
    endif()
  endif()
endif()

# Custom Starts from here
find_package(Eigen3 REQUIRED)

include_directories(
    include
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    ${EIGEN3_INCLUDE_DIR}
    ${RBDL_INCLUDE_DIR}
    DEPENDS system_lib
)
add_executable(${PROJECT_NAME} 
    src/main.cc
    # src/test_goal_Cd.cpp
    src/test.cpp
    src/quadraticprogram.cc
    ${SIMULATE_RESOURCE_FILES}
    ${RBDL_SOURCES}
    ${URDFREADER_SOURCES}
)
target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/libraries/qpOASES/include
  # ${CMAKE_SOURCE_DIR}/libraries/rbdl/include/rbdl
)
target_link_libraries(
  ${PROJECT_NAME}
  libsimulate
  mujoco::mujoco
  glfw
  Threads::Threads
  lodepng
  qpOASES
  # rbdl-static
)
target_compile_options(${PROJECT_NAME} PUBLIC ${MUJOCO_SIMULATE_COMPILE_OPTIONS})

if(APPLE AND MUJOCO_BUILD_MACOS_FRAMEWORKS)
  embed_in_bundle(${PROJECT_NAME})
endif()

# Do not install if macOS Bundles are created as RPATH is managed manually there.
if(APPLE AND MUJOCO_BUILD_MACOS_FRAMEWORKS)
  set(_INSTALL_SAMPLES OFF)
endif()