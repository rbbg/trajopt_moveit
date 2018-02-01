cmake_minimum_required(VERSION 2.8.3)
project(trajopt_moveit)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning
  cmake_modules
  pluginlib
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS moveit_ros_planning moveit_core cmake_modules pluginlib roscpp
#   DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/trajopt_planner.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# planner manager plugin
add_library(${PROJECT_NAME}_planner_manager
  src/trajopt_planner_manager.cpp)
target_link_libraries(${PROJECT_NAME}_planner_manager ${PROJECT_NAME} ${catkin_LIBRARIES})
