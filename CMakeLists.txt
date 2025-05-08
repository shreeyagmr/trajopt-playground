cmake_minimum_required(VERSION 3.16)
project(trajopt_playground)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find required packages
find_package(tesseract_common REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_kinematics REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_srdf REQUIRED)
find_package(tesseract_collision REQUIRED)
find_package(tesseract_motion_planners REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(console_bridge REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${tesseract_common_INCLUDE_DIRS}
  ${tesseract_environment_INCLUDE_DIRS}
  ${tesseract_kinematics_INCLUDE_DIRS}
  ${tesseract_scene_graph_INCLUDE_DIRS}
  ${tesseract_srdf_INCLUDE_DIRS}
  ${tesseract_collision_INCLUDE_DIRS}
  ${tesseract_motion_planners_INCLUDE_DIRS}
  ${tesseract_command_language_INCLUDE_DIRS}
  ${console_bridge_INCLUDE_DIRS}
  ${nlohmann_json_INCLUDE_DIRS}
)

# Add the source files
add_executable(trajopt_playground 
  src/main.cpp
)

# Link libraries
target_link_libraries(trajopt_playground
  ${tesseract_common_LIBRARIES}
  ${tesseract_environment_LIBRARIES}
  ${tesseract_kinematics_LIBRARIES}
  ${tesseract_scene_graph_LIBRARIES}
  ${tesseract_srdf_LIBRARIES}
  ${tesseract_collision_LIBRARIES}
  ${tesseract_motion_planners_LIBRARIES}
  ${tesseract_command_language_LIBRARIES}
  ${console_bridge_LIBRARIES}
  ${nlohmann_json_LIBRARIES}
)