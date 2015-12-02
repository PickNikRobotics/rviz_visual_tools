^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rviz_visual_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Contributors: Dave Coleman, Andy McEvoy, Jorge Cañardo Alastuey

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
