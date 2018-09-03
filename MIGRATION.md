# Migration Notes

API changes in Rviz Visual Tools releases

## ROS Melodic

- RvizVisualTools::publishMarkers called ros::spinOnce() in previous versions. This was usualy a bug as it might lead to running ros callback functions in unrelated threads (eg your UI thread). In the unlikely case that your programm relied on publishMarkers calling ros::spinOnce() you will now have to add either a ros::spinOnce after your call to publishMarkers or simply add a ros::AsyncSpinner. See http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning for more details.

