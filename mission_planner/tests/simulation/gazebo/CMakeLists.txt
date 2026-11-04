# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.16)
project(nomad_simulation LANGUAGES CXX)
find_package(gz-sim8 REQUIRED)
find_package(OpenCV REQUIRED)
add_library(NomadTestSystem SHARED NomadTestSystem.cc)
target_compile_features(NomadTestSystem PRIVATE cxx_std_17)
target_include_directories(NomadTestSystem PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(NomadTestSystem PRIVATE gz-sim8::gz-sim8 ${OpenCV_LIBS})
