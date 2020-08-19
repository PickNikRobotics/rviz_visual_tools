cmake_minimum_required(VERSION 3.0.2)
project(rviz_visual_tools)

if(NOT MSVC)
  add_compile_options(-std=c++11)
  # Warnings
  add_definitions(-W -Wall -Wextra
    #-Wcast-qual
    -Wwrite-strings -Wunreachable-code -Wpointer-arith
    -Winit-self -Wredundant-decls
    -Wno-unused-parameter -Wno-unused-function)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  eigen_stl_containers
  geometry_msgs
  graph_msgs
  interactive_markers
  shape_msgs
  roscpp
  roslint
  rostest
  rviz
  sensor_msgs
  std_msgs
  tf2_eigen
  tf2_ros
  tf2_geometry_msgs
  trajectory_msgs
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED thread system)
find_package(PkgConfig REQUIRED)
pkg_check_modules(OGRE_OV OGRE)

# Qt 4 or 5
if("${rviz_QT_VERSION}" VERSION_LESS "5")
  find_package(Qt4 ${rviz_QT_VERSION} REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
  macro(qt_wrap_ui)
    qt4_wrap_ui(${ARGN})
  endmacro()
else()
  find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
  macro(qt_wrap_ui)
    qt5_wrap_ui(${ARGN})
  endmacro()
endif()
## Prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOMOC_COMPILER_PREDEFINES OFF)
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
    ${PROJECT_NAME}_gui
    ${PROJECT_NAME}_remote_control
    ${PROJECT_NAME}_imarker_simple
  CATKIN_DEPENDS
    geometry_msgs
    graph_msgs
    interactive_markers
    shape_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf2_eigen
    tf2_ros
    trajectory_msgs
    visualization_msgs
  INCLUDE_DIRS
    include
    ${OGRE_OV_INCLUDE_DIRS}
)

include_directories(SYSTEM include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${OGRE_OV_INCLUDE_DIRS})

###########
## Build ##
###########

set(SOURCE_FILES
  src/${PROJECT_NAME}_gui.cpp
  src/key_tool.cpp
)

# Rviz GUI library
add_library(${PROJECT_NAME}_gui ${SOURCE_FILES} src/remote_control.cpp)
set_target_properties(${PROJECT_NAME}_gui PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${PROJECT_NAME}_gui ${rviz_DEFAULT_PLUGIN_LIBRARIES} ${QT_LIBRARIES} ${catkin_LIBRARIES})

# Remote control library
add_library(${PROJECT_NAME}_remote_control src/remote_control.cpp)
set_target_properties(${PROJECT_NAME}_remote_control PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${PROJECT_NAME}_remote_control ${catkin_LIBRARIES})

# Visualization Tools Library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
  src/tf_visual_tools.cpp
)
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_remote_control
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Library
add_library(${PROJECT_NAME}_imarker_simple
  src/imarker_simple.cpp
)
set_target_properties(${PROJECT_NAME}_imarker_simple PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${PROJECT_NAME}_imarker_simple
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PROJECT_NAME}
)

# Demo executable
add_executable(${PROJECT_NAME}_demo src/${PROJECT_NAME}_demo.cpp)
target_link_libraries(${PROJECT_NAME}_demo
  ${catkin_LIBRARIES} ${PROJECT_NAME}
)
set_target_properties(${PROJECT_NAME}_demo
  PROPERTIES
  OUTPUT_NAME
  demo PREFIX ""
)

# Demo executable
add_executable(${PROJECT_NAME}_imarker_simple_demo src/imarker_simple_demo.cpp)
target_link_libraries(${PROJECT_NAME}_imarker_simple_demo
  ${catkin_LIBRARIES} ${PROJECT_NAME} ${PROJECT_NAME}_imarker_simple
)
set_target_properties(${PROJECT_NAME}_imarker_simple_demo
  PROPERTIES
  OUTPUT_NAME
  imarker_simple_demo PREFIX ""
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
install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_gui
    ${PROJECT_NAME}_imarker_simple
    ${PROJECT_NAME}_remote_control
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION
    ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install shared resources
install(DIRECTORY launch    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY resources DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install xml plugin config
install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install executables
install(TARGETS ${PROJECT_NAME}_demo ${PROJECT_NAME}_imarker_simple_demo
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
