cmake_minimum_required(VERSION 2.8.3)
project(rviz_visual_tools)

add_definitions(-std=c++11)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Warnings
add_definitions(-W -Wall -Wextra
  #-Wcast-qual
  -Wwrite-strings -Wunreachable-code -Wpointer-arith
  -Winit-self -Wredundant-decls
  -Wno-unused-parameter -Wno-unused-function)

find_package(catkin REQUIRED COMPONENTS
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
  eigen_stl_containers
  rviz
  sensor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED thread system)

# Qt 4 or 5
if(rviz_QT_VERSION VERSION_LESS "5")
  find_package(Qt4 ${rviz_QT_VERSION} REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
  macro(qt_wrap_ui)
    qt4_wrap_ui(${ARGN})
  endmacro()
  macro(qt_wrap_cpp)
    qt4_wrap_cpp(${ARGN})
  endmacro()
else()
  find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
  macro(qt_wrap_ui)
    qt5_wrap_ui(${ARGN})
  endmacro()
  macro(qt_wrap_cpp)
    qt5_wrap_cpp(${ARGN})
  endmacro()
endif()
## Prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
    ${PROJECT_NAME}_gui
    ${PROJECT_NAME}_remote_control
    ${PROJECT_NAME}_imarker_simple
  CATKIN_DEPENDS
    eigen_conversions
    geometry_msgs
    visualization_msgs
    graph_msgs
    std_msgs
    tf_conversions
    trajectory_msgs
    sensor_msgs
    roscpp
  INCLUDE_DIRS
    include
)

include_directories(SYSTEM include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

###########
## Build ##
###########

## specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt_wrap_cpp(MOC_FILES
  src/${PROJECT_NAME}_gui.h
  src/key_tool.h
)

## specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/${PROJECT_NAME}_gui.cpp
  src/key_tool.cpp
)

# Rviz GUI library
add_library(${PROJECT_NAME}_gui ${SOURCE_FILES} src/remote_control.cpp)
target_link_libraries(${PROJECT_NAME}_gui ${rviz_DEFAULT_PLUGIN_LIBRARIES} ${QT_LIBRARIES} ${catkin_LIBRARIES})

# Remote control library
add_library(${PROJECT_NAME}_remote_control src/remote_control.cpp)
target_link_libraries(${PROJECT_NAME}_remote_control ${catkin_LIBRARIES})

# Visualization Tools Library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
  src/tf_visual_tools.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_remote_control
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Library
add_library(${PROJECT_NAME}_imarker_simple
  src/imarker_simple.cpp
)
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

# Demo executable
add_executable(imarker_simple_demo src/imarker_simple_demo.cpp)
target_link_libraries(imarker_simple_demo
  ${catkin_LIBRARIES} ${PROJECT_NAME} ${PROJECT_NAME}_imarker_simple
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
    ${PROJECT_NAME}_remote_control
    ${PROJECT_NAME}_imarker_simple
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
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
install(TARGETS ${PROJECT_NAME}_demo imarker_simple_demo
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
