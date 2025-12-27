cmake_minimum_required(VERSION 3.0.2)
project(Dome_master)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O3 -Wall")

# ===============================
# Catkin & ROS
# ===============================
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

# ===============================
# System dependencies
# ===============================
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP)
find_package(PkgConfig REQUIRED)

pkg_check_modules(FFTW REQUIRED fftw3)

# ===============================
# Catkin package
# ===============================
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  DEPENDS PCL Eigen3 OpenCV
)

# ===============================
# Include directories
# ===============================
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${FFTW_INCLUDE_DIRS}
)

# ===============================
# OpenMP
# ===============================
if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# ===============================
# Executable
# ===============================
add_executable(LinK3D_loop_test_dome
  src/dome_main.cpp
  src/dome_utils.cpp
)

add_dependencies(LinK3D_loop_test_dome
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(LinK3D_loop_test_dome
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${FFTW_LIBRARIES}
)