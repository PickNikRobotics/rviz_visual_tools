#!/usr/bin/env python3

import rviz_visual_tools as rvt
from rclpy.logging import get_logger
from geometry_msgs.msg import Point, Pose, Quaternion
from threading import Thread
import numpy as np
from copy import deepcopy
import rclcpp

LOGGER = get_logger("rviz_visual_tools_demo")

rclcpp.init()
node = rclcpp.Node("rviz_visual_tools_demo")
visual_tools = rvt.RvizVisualTools(node, "world", "/rviz_visual_tools")
visual_tools.load_marker_publisher(True)
visual_tools.delete_all_markers()
visual_tools.enable_batch_publishing(True)

point = Point(x=0.0, y=0.0, z=0.0)

step_x = 0.1

LOGGER.info("Displaying range of colors red->green")
visual_tools.publish_text(
    Pose(),
    "Shpere-Color-Range",
    rvt.Colors.WHITE,
    rvt.Scales.XXLARGE,
    False,
)
for color in (rvt.Colors.BLACK, rvt.Colors.GREY, rvt.Colors.WHITE, rvt.Colors.RED):
    visual_tools.publish_sphere(point, scale=rvt.Scales.XLARGE, color=color)
    point.x += step_x
visual_tools.trigger()


LOGGER.info("Displaying Coordinate Axis")
visual_tools.publish_text(
    Pose(position=Point(x=0.0, y=-0.2, z=0.0)),
    "Coordinate-Axis",
    rvt.Colors.WHITE,
    rvt.Scales.XXLARGE,
    False,
)
visual_tools.publish_axis(Pose(position=Point(x=0.0, y=-0.2, z=0.0)))
visual_tools.publish_axis(
    Pose(
        position=Point(x=0.25, y=-0.2, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=1.0, w=0.0),
    )
)
visual_tools.trigger()

LOGGER.info("Displaying Arrows")
visual_tools.publish_text(
    Pose(position=Point(x=0.0, y=-0.4, z=0.0)),
    "Arrows",
    rvt.Colors.WHITE,
    rvt.Scales.XXLARGE,
    False,
)
visual_tools.publish_x_arrow(
    Pose(
        position=Point(x=0.0, y=-0.4, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )
)
visual_tools.publish_y_arrow(
    Pose(
        position=Point(x=0.25, y=-0.4, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )
)
visual_tools.publish_z_arrow(
    Pose(
        position=Point(x=0.5, y=-0.4, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )
)
visual_tools.trigger()

cuboid_max_size = 0.075
cuboid_min_size = 0.01
point1 = Point(x=0.0, y=-0.6, z=0.0)
LOGGER.info("Displaying Rectangular Cuboid")
visual_tools.publish_text(
    Pose(position=point1), "Cuboid", rvt.Colors.WHITE, rvt.Scales.XXLARGE, False
)
for i in np.linspace(0, 1.0, 10):
    point2 = deepcopy(point1)
    point2.x += i * cuboid_max_size + cuboid_min_size
    point2.y += i * cuboid_max_size + cuboid_min_size
    point2.z += i * cuboid_max_size + cuboid_min_size
    visual_tools.publish_cuboid(point1, point2, color=rvt.Colors.RAND)
    point1.x += step_x
visual_tools.trigger()

line_max_size = 0.075
line_min_size = 0.01
point1 = Point(x=0.0, y=-0.8, z=0.0)
LOGGER.info("Displaying Lines")
visual_tools.publish_text(
    Pose(position=point1), "Line", rvt.Colors.WHITE, rvt.Scales.XXLARGE, False
)
for i in np.linspace(0, 1.0, 10):
    point2 = deepcopy(point1)
    point2.x += i * line_max_size + line_min_size
    point2.y += i * line_max_size + line_min_size
    point2.z += i * line_max_size + line_min_size
    visual_tools.publish_line(point1, point2, color=rvt.Colors.RAND)
    point1.x += step_x
visual_tools.trigger()

LOGGER.info("Displaying Cylinder")
visual_tools.publish_cylinder(
    Pose(position=Point(x=0.0, y=0.2, z=0.0)), color=rvt.Colors.RAND
)
visual_tools.trigger()

LOGGER.info("Displaying Cone")
visual_tools.publish_cone(
    Pose(position=Point(x=0.0, y=0.4, z=0.0)),
    angle=np.pi,
    color=rvt.Colors.RAND,
    scale=0.1,
)
visual_tools.trigger()

LOGGER.info("Displaying Planes")
plane_pose = Pose(position=Point(x=0.0, y=0.6, z=0.0))
visual_tools.publish_xy_plane(plane_pose, color=rvt.Colors.RED, scale=0.1)
visual_tools.publish_xz_plane(plane_pose, color=rvt.Colors.GREEN, scale=0.1)
visual_tools.publish_yz_plane(plane_pose, color=rvt.Colors.BLUE, scale=0.1)
visual_tools.trigger()

LOGGER.info("Displaying Labeled Coordinate Axis")
visual_tools.publish_axis_labeled(Pose(position=Point(x=0.0, y=0.8, z=0.0)), "MyAxis")
visual_tools.trigger()

LOGGER.info("Displaying Path")
visual_tools.publish_path(
    [Point(x=0.0, y=1.0, z=0.0), Point(x=0.2, y=1.0, z=0.0), Point(x=0.4, y=1.1, z=0.0)]
)
visual_tools.trigger()
