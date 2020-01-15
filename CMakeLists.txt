cmake_minimum_required(VERSION 3.5)
project(rviz_visual_tools)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-W -Wall -Wextra
    -Wwrite-strings -Wunreachable-code -Wpointer-arith
    -Winit-self -Wredundant-decls
    -Wno-unused-parameter -Wno-unused-function)
endif()

# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# TODO(mlautman): find alternatives to missing pkgs
# find_package(eigen_conversions)
# find_package(tf_conversions)
# find_package(graph_msgs REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(eigen_stl_containers REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED thread system)
find_package(PkgConfig REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

# pkg_check_modules(OGRE_OV OGRE)

# Qt 4 or 5
# if("${rviz_QT_VERSION}" VERSION_LESS "5")
#   find_package(Qt4 ${rviz_QT_VERSION} REQUIRED QtCore QtGui)
#   include(${QT_USE_FILE})
#   macro(qt_wrap_ui)
#     qt4_wrap_ui(${ARGN})
#   endmacro()
# else()
#   find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
#   set(QT_LIBRARIES Qt5::Widgets)
#   macro(qt_wrap_ui)
#     qt5_wrap_ui(${ARGN})
#   endmacro()
# endif()

## Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

## Prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
set(CMAKE_AUTOMOC_COMPILER_PREDEFINES OFF)
add_definitions(-DQT_NO_KEYWORDS)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

###########
## Build ##
###########
include_directories(include)

# Rviz GUI library
# set(SOURCE_FILES
  # src/${PROJECT_NAME}_gui.cpp
  # src/key_tool.cpp
# )
# add_library(${PROJECT_NAME}_gui ${SOURCE_FILES} src/remote_control.cpp)
# target_include_directories(${PROJECT_NAME}_gui PUBLIC
  # ${OGRE_INCLUDE_DIRS}
# )
# target_link_libraries(${PROJECT_NAME}_gui Qt5::Widgets)
# ament_target_dependencies(${PROJECT_NAME}_gui
  # rclcpp
# )

# Remote control library
add_library(${PROJECT_NAME}_remote_control src/remote_control.cpp)
target_link_libraries(${PROJECT_NAME}_remote_control)
ament_target_dependencies(${PROJECT_NAME}_remote_control
  rclcpp
)

# Visualization Tools Library
add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}.cpp
  src/tf_visual_tools.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_remote_control
  ${Boost_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
)

# Library
add_library(${PROJECT_NAME}_imarker_simple
  src/imarker_simple.cpp
)
target_link_libraries(${PROJECT_NAME}_imarker_simple
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}_imarker_simple
  rclcpp
)

# Demo executable
add_executable(${PROJECT_NAME}_demo src/${PROJECT_NAME}_demo.cpp)
target_link_libraries(${PROJECT_NAME}_demo
  ${PROJECT_NAME}
)
set_target_properties(${PROJECT_NAME}_demo
  PROPERTIES
  OUTPUT_NAME
  demo PREFIX ""
)

# Demo executable
add_executable(${PROJECT_NAME}_imarker_simple_demo src/imarker_simple_demo.cpp)
target_link_libraries(${PROJECT_NAME}_imarker_simple_demo
  ${PROJECT_NAME}
  ${PROJECT_NAME}_imarker_simple
)
set_target_properties(${PROJECT_NAME}_imarker_simple_demo
  PROPERTIES
  OUTPUT_NAME
  imarker_simple_demo PREFIX ""
)

##########
## TEST ##
##########

if (BUILD_TESTING)
  # Lint
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Run tests
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_rvt_test tests/rvt_test.cpp)
  ament_target_dependencies(${PROJECT_NAME}_rvt_test
    "rclcpp"
  )
  target_link_libraries(${PROJECT_NAME}_rvt_test
    ${PROJECT_NAME}
  )

endif()

## Test for correct C++ source code
# roslint_cpp()

#############
## Install ##
#############

ament_export_libraries(${PROJECT_NAME})
# ament_export_libraries(${PROJECT_NAME}_gui)
ament_export_libraries(${PROJECT_NAME}_remote_control)
ament_export_libraries(${PROJECT_NAME}_imarker_simple)

# Install libraries
install(
  TARGETS
    ${PROJECT_NAME}
    # ${PROJECT_NAME}_gui
    ${PROJECT_NAME}_imarker_simple
    ${PROJECT_NAME}_remote_control
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(
  TARGETS
    ${PROJECT_NAME}_demo
    ${PROJECT_NAME}_imarker_simple_demo
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install shared resources
install(DIRECTORY launch    DESTINATION share/${PROJECT_NAME})
install(DIRECTORY resources DESTINATION share/${PROJECT_NAME})

# Install xml plugin config
install(FILES
  plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include)

ament_export_include_directories(include)
ament_export_dependencies(
  # eigen_conversions
  # tf_conversions
  # graph_msgs
  geometry_msgs
  shape_msgs
  roscpp
  sensor_msgs
  std_msgs
  trajectory_msgs
  visualization_msgs
)

ament_package()