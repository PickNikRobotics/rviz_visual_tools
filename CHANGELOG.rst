^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rviz_visual_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
