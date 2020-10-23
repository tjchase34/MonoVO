cmake_minimum_required(VERSION 2.6)
project(MonoVO)

find_package(OpenCV REQUIRED)

add_executable(MonoVO src/vo.cpp)
add_executable(fg_test src/fg_test.cpp)

target_link_libraries(MonoVO ${OpenCV_LIBS} gtsam)
target_link_libraries(fg_test gtsam)
