cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(matplotlib_cpp LANGUAGES CXX)

include(GNUInstallDirs)
set(PACKAGE_NAME matplotlib_cpp)

# Library target
add_library(matplotlib_cpp INTERFACE)
target_compile_features(matplotlib_cpp INTERFACE
  cxx_std_11
)