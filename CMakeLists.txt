cmake_minimum_required(VERSION 2.8.3)
project(rviz_visual_tools)

# C++ 11
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  eigen_conversions
  geometry_msgs
  graph_msgs
  roscpp
  rostest
  std_msgs
  tf_conversions
  trajectory_msgs
  visualization_msgs
  roslint
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED thread system)

catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    geometry_msgs
    visualization_msgs
    graph_msgs
    std_msgs
    trajectory_msgs
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

###########
## Build ##
###########

# Visualization Tools Library
add_library(${PROJECT_NAME}
  src/rviz_visual_tools.cpp
  src/tf_visual_tools.cpp
)

add_dependencies(${PROJECT_NAME} graph_msgs_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

# Demo executable
add_executable(rviz_visual_tools_demo src/rviz_visual_tools_demo.cpp)
target_link_libraries(rviz_visual_tools_demo
  ${catkin_LIBRARIES} ${PROJECT_NAME}
)

##########
## TEST ##
##########

if(CATKIN_ENABLE_TESTING)

  add_rostest_gtest(rvt_test launch/rvt_test.launch tests/rvt_test.cpp)
  target_link_libraries(rvt_test
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
  )

endif()

## Test for correct C++ source code
roslint_cpp()

#############
## Install ##
#############

# Install libraries
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install shared resources
install(DIRECTORY launch    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY resources DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
