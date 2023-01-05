^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rviz_visual_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.1.4 (2023-01-05)
------------------
* Migrate to Ogre.h (`#226 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/226>`_)
* Remove galactic jobs since they are deprecated (`#232 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/232>`_)
* Add arbitrary color option to publishCuboid (`#227 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/227>`_)
* Bump clang-format version to 14 (`#228 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/228>`_)
* Contributors: Sebastian Castro, Stephanie Eng, Vatan Aksoy Tezer

4.1.3 (2022-07-18)
------------------
* Humble CI and formatting updates (`#220 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/220>`_)
* Minor typo fix (`#222 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/222>`_)
* Add a method to publish a plane using the normal and distance (`#221 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/221>`_)
  Co-authored-by: AndyZe <andyz@utexas.edu>
* Fix description of plane functions (`#219 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/219>`_)
* Update black version (`#218 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/218>`_)
* Add option to never wait for subscriber (`#217 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/217>`_)
* Contributors: Marq Rasmussen, Stephanie Eng, Vatan Aksoy Tezer

4.1.2 (2021-12-13)
------------------
* Fix faulty templated constructor (`#211 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/211>`_)
* Make sure to add all dependencies to the package.xml (`#209 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/209>`_)
  Otherwise it will fail to build on the buildfarm.
* Contributors: Chris Lalancette, Vatan Aksoy Tezer

4.1.1 (2021-10-07)
------------------
* Re-enable RemoteControl functionality (`#205 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/205>`_)
  * use condition_variable to be more thread safe
  * Drop executor from constructor, deprecate old one
  * Fix RemoteControl usage in demo
  * Use SystemDefaultsQOS for RemoteControl subscriber
  * Add RvizVisualToolsGui dashboard to rviz config, correct view
* Add pluginlib dependency. (`#203 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/203>`_)
* Fix package dependencies and cmake export (`#202 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/202>`_)
* Rename node_executable to executable (`#200 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/200>`_)
* Contributors: Davide Faconti, Henning Kayser, Jafar Abdi, Steven! Ragnarök, Vatan Aksoy Tezer

4.1.0 (2021-09-14)
------------------
* Fixes for new ros2 branch (`#198 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/198>`_)
* Fix include deprecation warning
* Enable Galactic and Rolling CI (`#190 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/190>`_)
  * minor compile fixes
* Fixes & improvements for deleting markers (`#188 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/188>`_)
  * Added RvizVisualTools method to delete all markers in a namespace
  * Fixed deleteAllMarkers for all namespaces
  * Added getters for marker ID's
* Move waitForSubscriber function to header file (`#185 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/185>`_)
* Contributors: Henning Kayser, Jafar Abdi, Nathan Brooks, Vatan Aksoy Tezer, Wyatt Rees

4.0.0 (2021-04-09)
------------------
* Fix warning about deprecation (`#180 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/180>`_)
  rclcpp::executor::FutureReturnCode was removed
* Update to offer shared_ptr to create_timer
* Fix rviz_visual_tools_gui (`#147 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/147>`_)
* Compilation for Windows (`#143 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/143>`_)
* Remove stl file in preparation for LFS setup (`#140 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/140>`_)
* Add function deleteMarker: deletes marker for given namespace and id (`#137 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/137>`_)
* fix waitForSubscriber time (`#142 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/142>`_)
* Eloquent cleanup (`#135 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/135>`_)
  * some misc cleanup of the imarker simple lib
  * actually reset marker counts
  * changes for using IMarker simple as a library
* Fix rviz warnings about scale and uninitialized quaternions (`#129 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/129>`_)
* Normalize interactive marker quaternions. (`#132 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/132>`_)
* normalize before publish (`#131 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/131>`_)
* New helper function getIdentityPose() (`#122 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/122>`_)
* For ABCD planes: better comments, ensure the plane equation is satisfied. (`#120 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/120>`_)
  Co-Authored-By: Henning Kayser <henningkayser@picknik.ai>
* Make `rviz_visual_tools` publish triangle mesh and `tf_visual_tools` clean published transforms (`#117 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/117>`_)
  * Add `clearAllTransform` method to tf_visual_tools
  * Add new function to publish mesh marker from triangles and vertices
* Publish plane from Ax+By+Cz+D=0 equation (`#119 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/119>`_)
* Fixed publishCylinder namespace (`#109 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/109>`_)
* Remove default arguments to make function calls not ambiguous (`#112 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/112>`_)
* Initialize quaternions in demo to avoid warning (`#111 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/111>`_)
* Fix node type in demo launch files (`#110 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/110>`_)
* Use LOGNAME for logging re:moveit styel (`#106 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/106>`_)
* Contributors: AndyZe, Bjar Ne, Dave Coleman, Jafar Abdi, Mike Lautman, Mori, MoriKen, Nathan Brooks, Sean Yen, Victor Lamoine, Yu Yan, d-walsh, hshose

3.7.0 (2018-11-26)
------------------
* Fix Eigen::Affine3d for Melodic (using Eigen::Isometry3d) (`#105 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/105>`_)
* Improve documentation (`#100 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/100>`_)
* Fix some catkin_lint warnings and rename targets output (`#98 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/98>`_)
* Update README Kinetic to Melodic (`#102 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/102>`_)
* Add Melodic build farm badges (`#99 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/99>`_)
* Add ccache support (`#93 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/93>`_)
* no ros::spinOnce() (`#82 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/82>`_)
* Deprecate old functions for ROS Melodic (`#91 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/91>`_)
* Improve package with catkin_lint (`#89 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/89>`_)
* Add OGRE dependency (`#88 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/88>`_)
* Automoc requires at least cmake version 2.8.6 (`#86 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/86>`_)
* Fix Travis badge (`#84 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/84>`_)
* The code already uses AUTOMOC (`#76 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/76>`_)
* Contributors: Dave Coleman, Jochen Sprickerhof, Michael Görner, Simon Schmeisser, Victor Lamoine

3.6.1 (2018-05-31)
------------------
* Actually link qt code into the gui library (`#74 <https://github.com/PickNikRobotics/rviz_visual_tools/issues/74>`_)
* Contributors: Michael Goerner

3.6.0 (2018-01-15)
------------------
* Addresses Issue #49 - Default Constructor Not Nodelet Friendly
* Added option to pass in a node handle in the constructor that defaults to
* Reset marker should publish initialized quaternion
* Improve code quality - add const, static, C++11 features, clang-format
* Create demo executable for IMarkerSimple
* Improve memory efficiency of functions
* Contributors: Dave Coleman, Geoffrey Chiou, Victor Lamoine

3.5.1 (2017-12-25)
------------------
* Normalize quaternion before storage
* Fix for ambiguous function call to publishAxis
* Add linking to visualization tools library for imarker\_simple
* Added arrow pub to take two points
* Document clang-tidy
* catkin lint
* roslint applied
* Clang-format again
* Clang-tidy ALL
* C++11 optimizations
* Fix deprecated calls to convertFromXYZRPY
* Add new convertPoseSafe() function
* New convertFromXYZRPY() function to avoid deprecation warning
* Allow to enable frame locking in the markers/markers arrays
* Update license year
* IMarkerSimple: set name and pose
* New printTransformFull() function
* Removed deprecated warning
* New class for easily using 6dof imarkers
* More options to tf\_visual\_tools
* Update README.md
* Contributors: Andy McEvoy, Dave Coleman, Fadri Furrer, Victor Lamoine

3.4.1 (2017-06-20)
------------------
* Add dependency on QT5 for Ubuntu Zesty/Lunar support
* Allow publishPath with std_msgs::ColorRGBA
* Make INFO msg DEBUG
* Contributors: Dave Coleman, Victor Lamoine

3.4.0 (2016-11-02)
------------------
* Consolidated publishing into RemoteReciever class
* Improve console output
* Add RvizGui and KeyTool
* Enable remote control from withing rviz_visual_tools
* New publishPath() function
* Shorten number of lines printTranslation() requires
* Contributors: Dave Coleman

3.3.0 (2016-09-28)
------------------
* BREAKING CHANGE: Make batch publishing enabled by default
* Removed enableInternalBatchPublishing()
* Removed triggerInternalBatchPublishAndDisable()
* Deprecated triggerBatchPublish() in favor of function name trigger()
* Deprecated triggerBatchPublishAndDisable()
* Ability to trigger every x markers that are in queue, ideal in for loops
* New waitForMarkerPub() function that takes timeout
* Add std::move
* Added Docker for Kinetic
* Added delay to demo to allow rviz to load in Docker
* Change the sphere marker type from SPHERE_LIST to SPHERE - This makes irregularly scaled spheres (i.e. ellipsoids) to be rendered correctly.
* Contributors: Dave Coleman, Miguel Prada

3.2.0 (2016-07-14)
------------------
* Catkin depend on eigen and tf conversions
* New warning
* Added EulerConvention enum
* Added new convertFromXYZRPY() function
* Added new tests
* Contributors: Dave Coleman, Enrique Fernandez

3.1.0 (2016-07-06)
------------------
* Switched publishPath() to use cylinders
* Added new publishLineStrip() function
* Added new publishPath() functions
* Added new publishAxis() functions
* Update screenshot
* Broke publishPath() API for recent addition - incorrect Eigen vector used
* New publishPath() function for Affine3d
* New publishAxis() functions that use scale
* New publishAxisInternal() function for more efficient publishing
* New publishAxisPath() function for showing a series of coordinate axis
* Added warning for batch publishing when not enabled
* Bug fix in publishLines() for id incrementing
* New scaleToString() function
* Bug fix for scaling in coordinate axis
* Improved demo to have multiple scales visualized
* Revert "Remove graph msgs"
* Contributors: Dave Coleman

3.0.0 (2016-06-29)
------------------
* Improve Travis script
* Upgrade to Eigen3 per ROS Kinetic requirements
* New screenshot
* Refactored entire scaling method - all sizes of shapes have been affected
* Renamed scales
* Removed const reference for primitive types in function headers
* Cleaned up getScale() function
* New publishSpheres function
* Two new tests & screenshot
* Deprecated size REGULAR
* Number scale sizes
* New intToRvizScale() function
* New publishSpheres() functions for showing list of lines AND colors
* New publishLines() functions for using LINE_LIST
* New publishCylinder() function that uses scales
* Bug fix for getVectorBetweenPoints() when vector is all zeros
* New printTranslation() function
* added intToRvizColor() for interfaces that do not directly depend on Rviz Visual Tools, such as OMPL
* publishPath with vector of colors
* Update demo
* New variant of publishPath with vector of colors
* New publishSphere() helper
* Add eigen_stl_containers
* Add missing breaks in switch statement
* Add namespace to ease debugging ROS messages
* Do not pass Eigen structures by value
* Fix all vector<Eigen> to EigenSTL::vector_Vector3d as recommended by @VictorLamoine
* Hide upstream package cast warning
* Overload new operator for Eigen structures
* Changing the angles will change the opening angle of the cone
* Better way to turn on C++11, maybe?
* Removed deprecated code for Kinetic
* Began converting to C++11
* Fix dead link to the documentation
* New waitForMarkerPub() function
* Fix bug in waitForSubscriber() introduced in previous commit
* Added blocking constraint option in function waitForSubscriber
* New publishLine() variant
* ID for publishing rectangles
* Optimize clearing and resizing vectors
* Increase random color sampling attempts
* Move variable declaration
* Latched publisher
* publishAxisLabeled arguments
* Include path, boost typedef and class name are now up to date with the code.
* New publishLine() variant
* Optional latched publisher
* ID for publishing rectangles
* Optimize clearing and resizing vectors
* Increase random color sampling attempts
* Move variable declaration
* publishAxisLabeled arguments order
* Adds Publish Labeled Axis
* Contributors: Abhijit Makhal, Dave Coleman, Naveed Usmani, Sow Papa Libasse, Victor Lamoine

2.2.0 (2016-04-28)
------------------
* Created much better demo, added new screenshot
* Numbered colors so that they can be matched in OMPL
* New publishLine() function variants
* Psychedelic mode
* Prevent publishing empty marker arrays
* Improved warning and error correction
* New publishSphere function
* Ability to set marker topic after constructor
* Ability to force waiting for topic to connect
* Added new posesEqual() function
* Updated publishArrow() function
* New publishPath function
* New publishLine function
* New publishCylinder that accepts two points
* New publishText function
* Removed redundant namespace names
* New convertPointToPose function
* Reduced output
* Renamed line_marker\_ to line_strip_marker\_
* Faster method for waiting for subscriber thread
* Untested publishPath() modification
* Fix to correctly use optional alpha color property
* Change getColorScale to work from 0->1 instead of 0->100
* Additional parameters to publishCuboid()
* New color scale function for generated interpolated colors from RED->GREEN (1->100)
* Contributors: Dave Coleman

2.1.0 (2016-02-09)
------------------
* Allow publishArrow functions to specify ID
* Contributors: Dave Coleman

2.0.3 (2016-01-10)
------------------
* Renamed test to demo
* Fix bug in random number generator
* Noted a TODO
* Documentation
* Contributors: Dave Coleman

2.0.2 (2015-12-27)
------------------
* Formatting
* Removed unused var
* roslint fixes
* Contributors: Dave Coleman

2.0.1 (2015-12-05)
------------------
* catkin lint cleanup
* Updated travis badge
* Updated README
* Contributors: Dave Coleman

2.0.0 (2015-12-02)
------------------
* Updated README
* Add badges
* Default true for enableBatchPublishing()
* Renamed convertXYZRPY() to convertFromXYZRPY()
* Changed roll pitch yall convention (fix)
* Added preliminary unit tests
* Hide include dependencies
* New convertToXYZRPY function
* Decrease wait time for topics to subscribe
* New publishSphere and publishArrow functions
* Added new thread safe pose conversion function
* Auto format with clang
* New publishSphere with frame_id function
* New print transform functions
* Fixed RPY error
* New convert Affine3d to roll pitch yaw function
* New tf_visual_tools functionality to help debug transforms
* New parameter server isEnabled feature
* Add id for wireframe cuboids
* Namespaced publishWireframeCuboid
* Helper function for publishAxisLabeled
* New getBoolMap() function
* New convertXYZRPY() function
* Fix warnings
* Fixed yellow
* Fix internal publish bug
* Check for empty parameter
* New delayed publishing internal mode
* added publishCuboid function for Eigen::Affine3d
* New string vector param reading
* added publishCuboid function for Eigen::Affine3d
* Show whole param path
* Added publish plane and cone
* Renamed to publishAxisLabled()
* New publishWireframeRectangle function
* Fixed publishZArrow direction
* New publishAxisWithLabel() function
* Removed mute functionality
* New publishWireframeRectangle() function
* Improved memory reuse by utilizing member variables for conversion functions
* Fixed ordering of functions in file
* Added alpha values to fix planning scene visualization
* Add WireframCuboid function to show oriented bounding boxes as computed ...
* Made more function parameters passed by reference
* Add color to wireframe
* Add WireframCuboid function to show oriented bounding boxes as computed from PCL.
* New generateRandomCuboid() function
* Fixed formatting, added a PoseStamped version to publish[X|Y|Z]Arrow() functions
* publishMesh() now has optional ID specification
* Fixed generateRandomPose() bug
* Added Eigen version of generateRandomPose()
* changed floats to double in random pose struct, added publish block function to take pose
* Updated rviz_visual_tools API
* Deprecated publishRectangle() in favor of publishCuboid()
* Added cyan and magenta
* Added maintainer
* Removed random pose bounds member variable in favor of using a funciton parameter
* Added publish arrow functions
* Added dark grey color
* New publishLine function takes two Vector3's
* added functionality to change bounds of random pose
* New publishArrow function that allows stamped pose for arbitrary parent frames
* added ArrayXXf to hold bounds on random pose
* new publishLine function takes two Vector3's
* Made yellow brighter
* added marker array to rviz and modified generate random pose to give actual random pose
* New publishArrow() functions
* New batch publishing method - allows markers to be published in batches to reduce ROS messages being published
* added method for displaying cuboids
* added a clear overlay
* New publishMesh function
* Added Brown, Pink, and Lime Green colors
* Copyright year
* Contributors: Dave Coleman, Andy McEvoy, Jorge Canardo Alastuey

1.5.0 (2015-01-07)
------------------
* New publishLine function
* New publishText() function with Eigen pose
* New publishAxis() feature
* New publishRectangle() functions
* New publishCylinder() functions
* New convertPoint() functions
* API: Renamed publishTest() to publishTests()
* Fix CMakeLists
* API Break: Change TRANSLUCENT2 to TRANSLUCENT_LIGHT
* New convertPoint() function
* New DEFAULT color to allow color selection to be disabled
* Fix install space
* Fix for publishRectangle() - zero scale size
* Added new size const values
* Contributors: Dave Coleman

1.4.0 (2014-10-31)
------------------
* Renamed VisualTools to RvizVisualTools
* Removed unnecessary dependency
* Bugfix
* Reduced debug output
* New waitForSubscriber() function that checks for first subscriber to a publisher
* New generateEmptyPose() function helper
* Consolidated publishing rviz messages to central publishMarker() function
* Contributors: Dave Coleman

1.3.1 (2014-10-27)
------------------
* Added new publishSpheres function
* Renamed rviz_colors to colors and rviz_scales to scales
* Initial commit, forked from moveit_visual_tools
* Contributors: Dave Coleman
