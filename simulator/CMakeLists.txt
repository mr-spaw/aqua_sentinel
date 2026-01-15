cmake_minimum_required(VERSION 3.16)
project(water_scada_sim LANGUAGES CXX)

# ---------------- C++ Standard ----------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ---------------- ROS 2 ----------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# ---------------- OpenGL ----------------
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)

# ---------------- jsoncpp ----------------
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

add_executable(sim sim.cpp)

target_include_directories(sim PRIVATE
  ${OPENGL_INCLUDE_DIRS}
  ${GLEW_INCLUDE_DIRS}
  ${GLUT_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)

target_link_libraries(sim
  ${OPENGL_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${JSONCPP_LIBRARIES}
  pthread
)

# 🔴 THIS LINE IS THE KEY
ament_target_dependencies(sim
  rclcpp
  std_msgs
)

ament_package()
