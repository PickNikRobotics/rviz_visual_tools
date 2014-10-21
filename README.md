Rviz Visual Tools
==========================

Helper functions for displaying and debugging data in Rviz via published markers.

This package includes:

 - Basic geometric markers for Rviz

Developed by [Dave Coleman](http://dav.ee) at the Correll Robotics Lab, University of Colorado Boulder with outside contributors.

<img align="right" src="https://raw.github.com/davetcoleman/rviz_visual_tools/indigo-devel/resources/screenshot.png" />

## Install

### Ubuntu Debian

```
sudo apt-get install ros-indigo-rviz-visual-tools
```

### Install From Source

Clone this repository into a catkin workspace, then use the rosdep install tool to automatically download its dependencies. Depending on your current version of ROS, use:
```
rosdep install --from-paths src --ignore-src --rosdistro indigo
```

## Quick Start

To see random shapes generated in Rviz:

    roslaunch rviz_visual_tools visual_tools_test.launch

	
## Code API

See [VisualTools Class Reference](http://docs.ros.org/indigo/api/rviz_visual_tools/html/classrviz__visual__tools_1_1VisualTools.html)

## Usage

We'll assume you will be using these helper functions within a class.

### Initialize

Add to your includes:
```
#include <rviz_visual_tools/visual_tools.h>
```

Add to your class's member variables:
```
// For visualizing things in rviz
rviz_visual_tools::VisualToolsPtr visual_tools_;
```

In your class' constructor add:
```
visual_tools_.reset(new rviz_visual_tools::VisualTools("base_frame","/rviz_visual_markers"));
```

Change the first parameter to the name of your robot's base frame, and the second parameter to whatever name you'd like to use for the corresponding Rviz marker ROS topic.

There are several other settings you can adjust, which I might get around to documenting in the future:
```
visual_tools_->setMuted(false);
visual_tools_->setLifetime(20.0);
visual_tools_->setFloorToBaseHeight(floor_to_base_height);
visual_tools_->setAlpha(alpha);
visual_tools_->setGlobalScale(scale);
visual_tools_->setBaseFrame(frame_name);
```

### Tools

Now in your code you can easily debug your code using visual markers in Rviz

Start rviz and create a new marker using the 'Add' button at the bottom right. Choose the marker topic to be the same as the topic you specified in the constructor.

### Example Code

In the following snippet we create a pose at xyz (0.1, 0.1, 0.1) and rotate the pose down 45 degrees along the Y axis. Then we publish the pose as a arrow for visualziation in Rviz. Make sure your Rviz fixed frame is the same as the one chosen in the code.

    // Create pose
    Eigen::Affine3d pose;
    pose = Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitY()); // rotate along X axis by 45 degrees
    pose.translation() = Eigen::Vector3d( 0.1, 0.1, 0.1 ); // translate x,y,z

    // Publish arrow vector of pose
    ROS_INFO_STREAM_NAMED("test","Publishing Arrow");
    visual_tools_->publishArrow(pose, rviz_visual_tools::RED, rviz_visual_tools::LARGE);


### Basic Publishing Functions

See ``visual_tools.h`` for more details and documentation on the following functions:

 - publishSphere
 - publishArrow
 - publishRectangle
 - publishLine
 - publishBlock
 - publishText
 - publishTest

And more...

### Helper Functions

Reset function

 - ``deleteAllMarkers`` - tells Rviz to clear out all current markers from being displayed. Only withs in ROS Indigo and newer.

Conversion functions

 - convertPose
 - convertPoint32ToPose
 - convertPoseToPoint
 - convertPoint
 - convertPoint32

Convenience functions

 - generateRandomPose
 - dRand
 - fRand
 - iRand
 - getCenterPoint
 - getVectorBetweenPoints

### Available Colors

This package helps you quickly choose colors - feel free to send PRs with more colors as needed

 - rviz_visual_tools::RED
 - rviz_visual_tools::GREEN
 - rviz_visual_tools::BLUE
 - rviz_visual_tools::GREY
 - rviz_visual_tools::WHITE
 - rviz_visual_tools::ORANGE
 - rviz_visual_tools::BLACK
 - rviz_visual_tools::YELLOW

### Available Marker Sizes

 - rviz_visual_tools::XXSMALL
 - rviz_visual_tools::XSMALL
 - rviz_visual_tools::SMALL
 - rviz_visual_tools::REGULAR
 - rviz_visual_tools::LARGE
 - rviz_visual_tools::XLARGE

### Lifetime

All markers will persist for the duration set by ``setLifetime``, defaulting to 30 seconds. You can reset this earlier by calling
```
resetMarkerCounts();
```
This will cause all new markers to overwrite older ones.

You can also delete all markers (new in ROS Indigo) by calling
```
deleteAllMarkers();
```

## Developers Notes

Useful notes for anyone wanting to dig in deeper:

 -  All poses are published with respect to the world frame e.g. /world, /odom, or maybe /base
 -  All publish() ROS topics should be followed by a ``ros::spinOnce();`` but no sleep
 -  Do not want to load any features/publishers until they are actually needed since this library contains so many components

## Contribute

Feel free to send PRs for new helper functions, fixes, etc. - I'll happily discuss and merge them. I do not, however, want to send much time helping people use this because I am a busy grad student. Use at your own risk.
