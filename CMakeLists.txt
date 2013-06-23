cmake_minimum_required(VERSION 2.8.3)
project(denso_rc7m_controller)

find_package(catkin REQUIRED COMPONENTS roscpp denso_bcap hardware_interface controller_manager)

# Declare catkin package
catkin_package(  
  INCLUDE_DIRS include
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}
  src/denso_rc7m_controller.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


