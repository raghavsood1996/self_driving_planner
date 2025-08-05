cmake_minimum_required(VERSION 3.10)
project(self_driving_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(self_driving_planner
    src/main.cpp
    src/Bitmap.cpp
    src/fssimplewindow.cpp
)

target_include_directories(self_driving_planner PRIVATE include)

# Link against OpenGL and GLUT; adjust as needed for target platform
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
target_link_libraries(self_driving_planner PRIVATE OpenGL::GL GLUT::GLUT)
