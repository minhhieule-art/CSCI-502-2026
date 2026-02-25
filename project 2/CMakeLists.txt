cmake_minimum_required(VERSION 3.10)
project(imu_glut CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(imu_glut
    src/main.cpp
    src/I2CBus.cpp
    src/MinIMU9.cpp
    src/MadgwickFilter.cpp
)

target_include_directories(imu_glut PRIVATE src)

# OpenGL + GLUT
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

target_link_libraries(imu_glut PRIVATE OpenGL::GL OpenGL::GLU GLUT::GLUT)