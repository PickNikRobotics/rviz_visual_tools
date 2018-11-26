# Migration Notes

API changes in Rviz Visual Tools releases

## ROS Melodic

- Affine3d no longer used, replaced by more computationally efficient Isometry3d. Simple find-replace should suffice
- RvizVisualTools::publishMarkers called ros::spinOnce() in previous versions. This was usually a bug as it might lead to running ROS callback functions in unrelated threads (e.g. your UI thread). In the unlikely case that your program relied on publishMarkers calling ros::spinOnce() you will now have to add either a ros::spinOnce after your call to publishMarkers or simply add a ros::AsyncSpinner. See http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning for more details.
