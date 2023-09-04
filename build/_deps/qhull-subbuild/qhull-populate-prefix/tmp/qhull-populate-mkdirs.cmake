# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/kist/euncheol/2DoFs_MPC/build/_deps/qhull-src"
  "/home/kist/euncheol/2DoFs_MPC/build/_deps/qhull-build"
  "/home/kist/euncheol/2DoFs_MPC/build/_deps/qhull-subbuild/qhull-populate-prefix"
  "/home/kist/euncheol/2DoFs_MPC/build/_deps/qhull-subbuild/qhull-populate-prefix/tmp"
  "/home/kist/euncheol/2DoFs_MPC/build/_deps/qhull-subbuild/qhull-populate-prefix/src/qhull-populate-stamp"
  "/home/kist/euncheol/2DoFs_MPC/build/_deps/qhull-subbuild/qhull-populate-prefix/src"
  "/home/kist/euncheol/2DoFs_MPC/build/_deps/qhull-subbuild/qhull-populate-prefix/src/qhull-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/kist/euncheol/2DoFs_MPC/build/_deps/qhull-subbuild/qhull-populate-prefix/src/qhull-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/kist/euncheol/2DoFs_MPC/build/_deps/qhull-subbuild/qhull-populate-prefix/src/qhull-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
