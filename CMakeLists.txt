cmake_minimum_required(VERSION 2.6)
project(MonoVO)

find_package(OpenCV REQUIRED)

add_executable(MonoVO src/vo.cpp)

target_link_libraries(MonoVO ${OpenCV_LIBS})