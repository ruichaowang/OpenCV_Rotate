cmake_minimum_required(VERSION 3.22.0)
project(Rotate_convert VERSION 0.1.0)
# 设定读取的地址.

# find required opencv
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc videoio calib3d PATHS /opt/homebrew/Cellar/opencv/4.5.5)
# directory of opencv headers
include_directories(${OpenCV_INCLUDE_DIRS})

# directory of opencv library
link_directories(${OpenCV_LIBRARY_DIRS})

message(STATUS "找到的 OpenCV_INCLUDE_DIRS 地址: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "找到的 OpenCV_LIBS : ${OpenCV_LIBS}" )

add_executable("Rotate_convert" main.cpp include/common_function.hpp)
target_link_libraries(Rotate_convert ${OpenCV_LIBS})
# opencv libraries

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)






