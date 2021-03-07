cmake_minimum_required(VERSION 2.8)

# define project
project(QuadControl)

# set include directory
include_directories(include)

# search for all c files in source folder
file(GLOB SOURCES "src/*.c")

# make executable
add_executable(QuadControl ${SOURCES}) 

# add necessary libraries (multithreading, math, ftdi) for linking
TARGET_LINK_LIBRARIES(QuadControl pthread m ftd2xx) 
