cmake_minimum_required(VERSION 3.20)
project(cpp_src)

# add boost lib
set(CMAKE_CXX_STANDARD 11)
set(BOOST_ROOT /usr/local/include/boost)
find_package(Boost COMPONENTS system thread regex serialization REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
MESSAGE( STATUS "Boost_INCLUDE_DIRS = ${Boost_INCLUDE_DIRS}.")
MESSAGE( STATUS "Boost_LIBRARIES = ${Boost_LIBRARIES}.")
MESSAGE( STATUS "Boost_LIB_VERSION = ${Boost_LIB_VERSION}.")

# add opencv lib
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(cpp_src main.cpp serial_pc2mcu.cpp serial_pc2mcu.h) # laser_find_test.cpp sys_test.cpp
target_link_libraries(cpp_src ${OpenCV_LIBS} ${Boost_LIBRARIES})





