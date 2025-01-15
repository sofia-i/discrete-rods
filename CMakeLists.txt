cmake_minimum_required(VERSION 3.17)
project(discrete-rods)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Find Eigen path to include
find_path(EIGEN_DIR NAMES signature_of_eigen3_matrix_library
        PATHS
        /opt/homebrew/Cellar/eigen
#        /usr/local/include/eigen3
        PATH_SUFFIXES eigen3 eigen)
if(NOT EXISTS ${EIGEN_DIR})
    message(FATAL_ERROR "Please check that the set Eigen directory is valid!")
endif()
include_directories(${EIGEN_DIR})

# Configure executables
add_executable(discrete-rods src/main.cpp src/Simulator.cpp src/Rod.cpp
        src/VectorProperty.cpp
        src/MathUtilities.h
        src/MatrixProperty.cpp)