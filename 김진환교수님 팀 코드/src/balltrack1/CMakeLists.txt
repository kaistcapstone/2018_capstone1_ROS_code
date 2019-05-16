cmake_minimum_required(VERSION 2.8)

project(balltrack1)
add_executable(${PROJECT_NAME} "main.cpp")

find_package(OpenCV REQUIRED)
target_link_libraries(balltrack1 ${OpenCV_LIBS})
