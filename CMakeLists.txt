cmake_minimum_required(VERSION 2.8.3)
project(my_vo)

ADD_COMPILE_OPTIONS(-std=c++17 )
set( CMAKE_CXX_FLAGS "-std=c++17 -O3" )
set(CMAKE_C_FLAGS_DEBUG "-g -DDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-g -DDEBUG")

find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)

set(BOOST_LIBS date_time system program_options filesystem)
find_package(Boost COMPONENTS ${BOOST_LIBS} REQUIRED)

find_package(yaml-cpp REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    include
)

add_executable(main src/main.cpp src/my_VO.cpp include/config.cpp)
target_link_libraries(main ${OpenCV_LIBS} ${Boost_LIBRARIES} yaml-cpp)