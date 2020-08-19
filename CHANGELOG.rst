^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rviz_visual_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.9.1 (2020-10-09)
------------------
* [maint] add soname versions to libraries (`#166 <https://github.com/tylerjw/rviz_visual_tools/issues/166>`_)
* [maint] Apply clang-format-10 (`#173 <https://github.com/tylerjw/rviz_visual_tools/issues/173>`_)
* Contributors: Tyler Weaver

3.9.0 (2020-09-05)
------------------
* [feature] Add optional parent frame to iMarker (`#159 <https://github.com/tylerjw/rviz_visual_tools/issues/159>`_)
* [feature] Publish cuboid with size as a vector3 or Eigen::Vector3d  (`#125 <https://github.com/tylerjw/rviz_visual_tools/issues/125>`_)
* [feature] Normalize interactive marker quaternions. (`#132 <https://github.com/tylerjw/rviz_visual_tools/issues/132>`_)
* [feature] normalize before publish (`#131 <https://github.com/tylerjw/rviz_visual_tools/issues/131>`_)
* [feature] New helper function getIdentityPose() (`#122 <https://github.com/tylerjw/rviz_visual_tools/issues/122>`_)
* [feature] Windows bring up. (`#116 <https://github.com/tylerjw/rviz_visual_tools/issues/116>`_)
* [feature] For ABCD planes: better comments, ensure the plane equation is satisfied. (`#120 <https://github.com/tylerjw/rviz_visual_tools/issues/120>`_)
* [feature] Make `rviz_visual_tools` publish triangle mesh and `tf_visual_tools` clean published transforms (`#117 <https://github.com/tylerjw/rviz_visual_tools/issues/117>`_)
* [feature] Publish plane from Ax+By+Cz+D=0 equation (`#119 <https://github.com/tylerjw/rviz_visual_tools/issues/119>`_)
* [feature] Remove default arguments to make function calls not ambiguous (`#112 <https://github.com/tylerjw/rviz_visual_tools/issues/112>`_)
* [feature] Initizalize quaternions in demo to avoid warning (`#111 <https://github.com/tylerjw/rviz_visual_tools/issues/111>`_)
* [fix] rviz warnings about scale and uninitialized quaternions (`#129 <https://github.com/tylerjw/rviz_visual_tools/issues/129>`_)
* [fix] publishCylinder namespace (`#109 <https://github.com/tylerjw/rviz_visual_tools/issues/109>`_)
* [fix] node type in demo launch files (`#110 <https://github.com/tylerjw/rviz_visual_tools/issues/110>`_)
* [maint] Adding missing dependency (interactive_markers) (`#168 <https://github.com/tylerjw/rviz_visual_tools/issues/168>`_)
* [maint] clang-tidy (`#158 <https://github.com/tylerjw/rviz_visual_tools/issues/158>`_)
* [maint] replace tf_conversions with tf2 (`#151 <https://github.com/tylerjw/rviz_visual_tools/issues/151>`_)
* [maint] bump cmake version (`#150 <https://github.com/tylerjw/rviz_visual_tools/issues/150>`_)
* [maint] remove trailing whitespaces (`#130 <https://github.com/tylerjw/rviz_visual_tools/issues/130>`_)
* [maint] Apply clang-tidy (`#127 <https://github.com/tylerjw/rviz_visual_tools/issues/127>`_)
* [maint] Switch to moveit_ci, apply clang-format (`#124 <https://github.com/tylerjw/rviz_visual_tools/issues/124>`_)
* [maint] Use LOGNAME for logging re:moveit styel (`#106 <https://github.com/tylerjw/rviz_visual_tools/issues/106>`_)
* Contributors: AndyZe, Bjar Ne, Dave Coleman, Jafar Abdi, JafarAbdi, Michael Görner, Mike Lautman, Sean Yen, Victor Lamoine, Yu, Yan, d-walsh

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
