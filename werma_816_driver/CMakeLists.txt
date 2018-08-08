cmake_minimum_required(VERSION 2.8.3)
project(werma_816_driver)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  werma_816_msgs
)

catkin_package(CATKIN_DEPENDS roscpp)
include_directories(${catkin_INCLUDE_DIRS})
add_executable(werma_816_driver src/main.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(werma_816_driver ${catkin_LIBRARIES})
