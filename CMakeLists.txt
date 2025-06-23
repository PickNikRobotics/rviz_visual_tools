cmake_minimum_required(VERSION 3.20)
project(rviz_visual_tools)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-W -Wall -Wextra
    -Wwrite-strings -Wunreachable-code -Wpointer-arith
    -Winit-self -Wredundant-decls
    -Wno-unused-parameter -Wno-unused-function)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
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

find_package(PkgConfig REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(pluginlib REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_default_plugins REQUIRED)


find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Test  Widgets)

if(Qt${QT_VERSION_MAJOR} VERSION_EQUAL "5")
	find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)
	if(${QT_VERSION} VERSION_LESS 5.15.0)
  		function(qt_wrap_cpp out)
    		qt5_wrap_cpp(_sources ${ARGN})
    		set("${out}" ${_sources} PARENT_SCOPE)
  		endfunction()
	endif()
elseif(Qt${QT_VERSION_MAJOR} STREQUAL "6")
  find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets Core5Compat)
endif()


## Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)


## Prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
#set(CMAKE_AUTOMOC_COMPILER_PREDEFINES OFF)
#add_definitions(-DQT_NO_KEYWORDS)

###########
## Build ##
###########

# Needed for shared libraries on Windows
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
if(NOT DEFINED BUILD_SHARED_LIBS)
  option(BUILD_SHARED_LIBS "Build dynamically-linked binaries" ON)
endif()


 
add_library(${PROJECT_NAME}_gui SHARED src/${PROJECT_NAME}_gui.cpp
   src/key_tool.cpp
   src/remote_control.cpp )#${HEADER_FILES})

target_include_directories(
  ${PROJECT_NAME}_gui
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)

if(Qt${QT_VERSION_MAJOR} VERSION_EQUAL "6")
  target_link_libraries(${PROJECT_NAME}_gui PUBLIC
  ${rclcpp_components_TARGETS}
  rclcpp::rclcpp
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  rviz_default_plugins::rviz_default_plugins
  rviz_ogre_vendor::OgreMain rviz_ogre_vendor::OgreOverlay 
  Qt${QT_VERSION_MAJOR}::Widgets Qt${QT_VERSION_MAJOR}::Core5Compat
  )
else()
  target_link_libraries(${PROJECT_NAME}_gui PUBLIC
  ${rclcpp_components_TARGETS}
  rclcpp::rclcpp
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  rviz_default_plugins::rviz_default_plugins
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  Qt${QT_VERSION_MAJOR}::Widgets
)
endif()


# prevent pluginlib from using boost
#target_compile_definitions(${PROJECT_NAME}_gui PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
## Causes the visibility macros to use dllexport rather than dllimport (for Windows, when your plugin should be used as library)
#target_compile_definitions(${PROJECT_NAME}_gui PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

add_library(${PROJECT_NAME}_remote_control SHARED
  src/remote_control.cpp
)

target_include_directories(
  ${PROJECT_NAME}_remote_control
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         )#${BULLET_INCLUDE_DIRS})

# Needed for M_PI on Windows
target_compile_definitions(${PROJECT_NAME}_remote_control PRIVATE _USE_MATH_DEFINES)

target_link_libraries(${PROJECT_NAME}_remote_control
  ${rclcpp_components_TARGETS}
  ${visualization_msgs_TARGETS}
  tf2::tf2
  tf2_eigen::tf2_eigen
  ${tf2_geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${shape_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${trajectory_msgs_TARGETS}
  eigen_stl_containers::eigen_stl_containers
)

# Visualization Tools Library
add_library(${PROJECT_NAME} SHARED
  src/rviz_visual_tools.cpp
  src/tf_visual_tools.cpp
)
target_compile_definitions(${PROJECT_NAME} PRIVATE _USE_MATH_DEFINES)

set(${PROJECT_NAME}_LINK_LIBRARIES
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  rviz_common::rviz_common
  sensor_msgs::sensor_msgs_library
  tf2_geometry_msgs::tf2_geometry_msgs
  Qt${QT_VERSION_MAJOR}::Widgets
)

if(Qt${QT_VERSION_MAJOR} VERSION_EQUAL "6")
  list(APPEND ${PROJECT_NAME}_LINK_LIBRARIES
    Qt6::Core5Compat
  )
endif()

target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_remote_control
    ${PROJECT_NAME}_LINK_LIBRARIES
)


# Library
add_library(${PROJECT_NAME}_imarker_simple SHARED
  src/imarker_simple.cpp
)
target_compile_definitions(${PROJECT_NAME}_imarker_simple PRIVATE _USE_MATH_DEFINES)
target_link_libraries(${PROJECT_NAME}_imarker_simple
  ${PROJECT_NAME}
  rclcpp::rclcpp
  interactive_markers::interactive_markers
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  eigen_stl_containers::eigen_stl_containers
  Eigen3::Eigen
  Qt${QT_VERSION_MAJOR}::Widgets
)

# Demo executable
add_executable(${PROJECT_NAME}_demo
  src/${PROJECT_NAME}_demo.cpp
)
target_compile_definitions(${PROJECT_NAME}_demo PRIVATE _USE_MATH_DEFINES)
target_link_libraries(${PROJECT_NAME}_demo
  ${PROJECT_NAME}
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

# Demo executable
add_executable(${PROJECT_NAME}_imarker_simple_demo
  src/imarker_simple_demo.cpp)
target_compile_definitions(${PROJECT_NAME}_imarker_simple_demo PRIVATE _USE_MATH_DEFINES)
target_link_libraries(${PROJECT_NAME}_imarker_simple_demo
  ${PROJECT_NAME}_imarker_simple
  rclcpp::rclcpp
)

#############
## Install ##
#############
# Install libraries
install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_remote_control
    ${PROJECT_NAME}_gui
    ${PROJECT_NAME}_imarker_simple
  EXPORT
    export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
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

# Install header files
install(DIRECTORY include/ DESTINATION include)

# Install shared resources
install(DIRECTORY launch    DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(geometry_msgs
  shape_msgs
  rclcpp
  sensor_msgs
  std_msgs
  trajectory_msgs
  visualization_msgs
  eigen3_cmake_module
  Eigen3
  eigen_stl_containers
  interactive_markers
  rviz_common
  rviz_rendering
  rviz_default_plugins
  rviz_ogre_vendor
)
ament_export_include_directories(include)
ament_export_libraries(
  ${PROJECT_NAME}
  ${PROJECT_NAME}_remote_control
  ${PROJECT_NAME}_imarker_simple
  ${PROJECT_NAME}_gui
)

##########
## TEST ##
##########
if (BUILD_TESTING)
  # Run tests
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_rvt_test
    tests/rvt_test.cpp
    TIMEOUT 180)
  target_compile_definitions(${PROJECT_NAME}_rvt_test PRIVATE _USE_MATH_DEFINES)
  target_link_libraries(${PROJECT_NAME}_rvt_test
    ${PROJECT_NAME}
    rclcpp::rclcpp
  )
endif()

ament_package()
