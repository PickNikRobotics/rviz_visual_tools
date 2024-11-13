#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <py_binding_tools/ros_msg_typecasters.h>

namespace py = pybind11;
using py::literals::operator""_a;

namespace rviz_visual_tools
{
PYBIND11_MODULE(pyrviz_visual_tools, m)
{
  py::enum_<Colors>(m, "Colors")
      .value("BLACK", Colors::BLACK)
      .value("BROWN", Colors::BROWN)
      .value("BLUE", Colors::BLUE)
      .value("CYAN", Colors::CYAN)
      .value("GREY", Colors::GREY)
      .value("DARK_GREY", Colors::DARK_GREY)
      .value("GREEN", Colors::GREEN)
      .value("LIME_GREEN", Colors::LIME_GREEN)
      .value("MAGENTA", Colors::MAGENTA)
      .value("ORANGE", Colors::ORANGE)
      .value("PURPLE", Colors::PURPLE)
      .value("RED", Colors::RED)
      .value("PINK", Colors::PINK)
      .value("WHITE", Colors::WHITE)
      .value("YELLOW", Colors::YELLOW)
      .value("TRANSLUCENT", Colors::TRANSLUCENT)
      .value("TRANSLUCENT_LIGHT", Colors::TRANSLUCENT_LIGHT)
      .value("TRANSLUCENT_DARK", Colors::TRANSLUCENT_DARK)
      .value("RAND", Colors::RAND)
      .value("CLEAR", Colors::CLEAR)
      .value("DEFAULT", Colors::DEFAULT);

  py::enum_<Scales>(m, "Scales")
      .value("XXXXSMALL", Scales::XXXXSMALL)
      .value("XXXSMALL", Scales::XXXSMALL)
      .value("XXSMALL", Scales::XXSMALL)
      .value("XSMALL", Scales::XSMALL)
      .value("SMALL", Scales::SMALL)
      .value("MEDIUM", Scales::MEDIUM)
      .value("LARGE", Scales::LARGE)
      .value("XLARGE", Scales::XLARGE)
      .value("XXLARGE", Scales::XXLARGE)
      .value("XXXLARGE", Scales::XXXLARGE)
      .value("XXXXLARGE", Scales::XXXXLARGE);

  py::enum_<EulerConvention>(m, "EulerConvention")
      .value("XYZ", EulerConvention::XYZ)
      .value("ZYX", EulerConvention::ZYX)
      .value("ZXZ", EulerConvention::ZXZ);

  py::class_<RvizVisualTools>(m, "RvizVisualTools")
      .def(py::init([](const rclcpp::Node::SharedPtr& node, const std::string& base_frame,
                       const std::string& marker_topic) {
        return RvizVisualTools(base_frame, marker_topic, node);
      }))
      .def("delete_marker", &RvizVisualTools::deleteMarker)
      .def("delete_all_markers", py::overload_cast<>(&RvizVisualTools::deleteAllMarkers))
      .def("delete_all_markers",
           py::overload_cast<const std::string&>(&RvizVisualTools::deleteAllMarkers))
      .def("reset_marker_counts", &RvizVisualTools::resetMarkerCounts)
      .def("load_rviz_markers", &RvizVisualTools::loadRvizMarkers)
      .def("set_marker_topic", &RvizVisualTools::setMarkerTopic)
      .def("load_marker_publisher", &RvizVisualTools::loadMarkerPub)
      .def("wait_for_marker_subscriber", &RvizVisualTools::waitForMarkerSub)
      .def("set_alpha", &RvizVisualTools::setAlpha)
      .def("set_lifetime", &RvizVisualTools::setLifetime)
      .def_static("get_random_color", &RvizVisualTools::getRandColor)
      .def("get_color", &RvizVisualTools::getColor)
      .def("get_color_scale", &RvizVisualTools::getColorScale)
      .def("get_scale", &RvizVisualTools::getScale)
      .def_property("base_frame", &RvizVisualTools::getBaseFrame, &RvizVisualTools::setBaseFrame)
      .def_property("global_scale", &RvizVisualTools::getGlobalScale,
                    &RvizVisualTools::setGlobalScale)
      .def("enable_batch_publishing", &RvizVisualTools::enableBatchPublishing, "enable"_a = true)
      .def("enable_frame_locking", &RvizVisualTools::enableFrameLocking)
      .def("trigger_every", &RvizVisualTools::triggerEvery)
      .def("trigger", &RvizVisualTools::trigger)
      .def("publish_markers", &RvizVisualTools::publishMarkers)
      .def("publish_cone",
           py::overload_cast<const geometry_msgs::msg::Pose&, double, Colors, double>(
               &RvizVisualTools::publishCone),
           "pose"_a, "angle"_a, "color"_a = Colors::TRANSLUCENT, "scale"_a = 1.0)
      .def("publish_abcd_plane", &RvizVisualTools::publishABCDPlane, "A"_a, "B"_a, "C"_a, "D"_a,
           "color"_a = Colors::TRANSLUCENT, "x_width"_a = 1.0, "y_width"_a = 1.0)
      .def("publish_normal_and_distance_plane", &RvizVisualTools::publishNormalAndDistancePlane,
           "normal"_a, "distance"_a, "color"_a = Colors::TRANSLUCENT, "x_width"_a = 1.0,
           "y_width"_a = 1.0)
      .def("publish_xy_plane",
           py::overload_cast<const geometry_msgs::msg::Pose&, Colors, double>(
               &RvizVisualTools::publishXYPlane),
           "pose"_a, "color"_a = Colors::TRANSLUCENT, "scale"_a = 1.0)
      .def("publish_xz_plane",
           py::overload_cast<const geometry_msgs::msg::Pose&, Colors, double>(
               &RvizVisualTools::publishXZPlane),
           "pose"_a, "color"_a = Colors::TRANSLUCENT, "scale"_a = 1.0)
      .def("publish_yz_plane",
           py::overload_cast<const geometry_msgs::msg::Pose&, Colors, double>(
               &RvizVisualTools::publishYZPlane),
           "pose"_a, "color"_a = Colors::TRANSLUCENT, "scale"_a = 1.0)
      .def("publish_sphere",
           py::overload_cast<const geometry_msgs::msg::Point&, Colors, Scales, const std::string&,
                             std::size_t>(&RvizVisualTools::publishSphere),
           "point"_a, "color"_a = Colors::BLUE, "scale"_a = Scales::MEDIUM, "ns"_a = "Sphere",
           "id"_a = 0)
      .def("publish_sphere",
           py::overload_cast<const geometry_msgs::msg::Pose&, Colors, Scales, const std::string&,
                             std::size_t>(&RvizVisualTools::publishSphere),
           "pose"_a, "color"_a = Colors::BLUE, "scale"_a = Scales::MEDIUM, "ns"_a = "Sphere",
           "id"_a = 0)
      .def("publish_spheres",
           py::overload_cast<const std::vector<geometry_msgs::msg::Point>&, Colors, Scales,
                             const std::string&>(&RvizVisualTools::publishSpheres),
           "points"_a, "color"_a = Colors::BLUE, "scale"_a = Scales::MEDIUM, "ns"_a = "Spheres")
      .def("publish_x_arrow",
           py::overload_cast<const geometry_msgs::msg::Pose&, Colors, Scales, double>(
               &RvizVisualTools::publishXArrow),
           "pose"_a, "color"_a = Colors::RED, "scale"_a = Scales::MEDIUM, "length"_a = 0.0)
      .def("publish_x_arrow",
           py::overload_cast<const geometry_msgs::msg::PoseStamped&, Colors, Scales, double>(
               &RvizVisualTools::publishXArrow),
           "pose_stamped"_a, "color"_a = Colors::RED, "scale"_a = Scales::MEDIUM, "length"_a = 0.0)
      .def("publish_y_arrow",
           py::overload_cast<const geometry_msgs::msg::Pose&, Colors, Scales, double>(
               &RvizVisualTools::publishYArrow),
           "pose"_a, "color"_a = Colors::GREEN, "scale"_a = Scales::MEDIUM, "length"_a = 0.0)
      .def("publish_y_arrow",
           py::overload_cast<const geometry_msgs::msg::PoseStamped&, Colors, Scales, double>(
               &RvizVisualTools::publishYArrow),
           "pose_stamped"_a, "color"_a = Colors::GREEN, "scale"_a = Scales::MEDIUM,
           "length"_a = 0.0)
      .def("publish_z_arrow",
           py::overload_cast<const geometry_msgs::msg::Pose&, Colors, Scales, double>(
               &RvizVisualTools::publishZArrow),
           "pose"_a, "color"_a = Colors::BLUE, "scale"_a = Scales::MEDIUM, "length"_a = 0.0)
      .def("publish_z_arrow",
           py::overload_cast<const geometry_msgs::msg::PoseStamped&, Colors, Scales, double>(
               &RvizVisualTools::publishZArrow),
           "pose_stamped"_a, "color"_a = Colors::BLUE, "scale"_a = Scales::MEDIUM, "length"_a = 0.0)
      .def("publish_cuboid",
           py::overload_cast<const geometry_msgs::msg::Point&, const geometry_msgs::msg::Point&,
                             Colors, const std::string&, std::size_t>(
               &RvizVisualTools::publishCuboid),
           "point1"_a, "point2"_a, "color"_a = Colors::BLUE, "ns"_a = "Cuboid", "id"_a = 0)
      .def("publish_cuboid",
           py::overload_cast<const geometry_msgs::msg::Pose&, double, double, double, Colors>(
               &RvizVisualTools::publishCuboid),
           "pose"_a, "depth"_a, "width"_a, "height"_a, "color"_a = Colors::BLUE)
      .def("publish_line",
           py::overload_cast<const geometry_msgs::msg::Point&, const geometry_msgs::msg::Point&,
                             Colors, Scales>(&RvizVisualTools::publishLine),
           "point1"_a, "point2"_a, "color"_a = Colors::BLUE, "scale"_a = Scales::MEDIUM)
      .def("publish_path",
           py::overload_cast<const std::vector<geometry_msgs::msg::Point>&, Colors, Scales,
                             const std::string&>(&RvizVisualTools::publishPath),
           "path"_a, "color"_a = Colors::RED, "scale"_a = Scales::MEDIUM, "ns"_a = "Path")
      .def("publish_polygon", &RvizVisualTools::publishPolygon, "polygon"_a,
           "color"_a = Colors::RED, "scale"_a = Scales::MEDIUM, "ns"_a = "Polygon")
      .def("publish_axis_labeled",
           py::overload_cast<const geometry_msgs::msg::Pose&, const std::string&, Scales, Colors>(
               &RvizVisualTools::publishAxisLabeled),
           "pose"_a, "label"_a, "scale"_a = Scales::MEDIUM, "color"_a = Colors::WHITE)
      .def("publish_axis",
           py::overload_cast<const geometry_msgs::msg::Pose&, Scales, const std::string&>(
               &RvizVisualTools::publishAxis),
           "pose"_a, "scale"_a = Scales::MEDIUM, "ns"_a = "Axis")
      .def("publish_cylinder",
           py::overload_cast<const geometry_msgs::msg::Pose&, Colors, double, double,
                             const std::string&>(&RvizVisualTools::publishCylinder),
           "pose"_a, "color"_a = Colors::BLUE, "height"_a = 0.1, "radius"_a = 0.01,
           "ns"_a = "Cylinder")
      .def("publish_mesh",
           py::overload_cast<const geometry_msgs::msg::Pose&, const std::string&, Colors, double,
                             const std::string&, std::size_t>(&RvizVisualTools::publishMesh),
           "pose"_a, "filename"_a, "color"_a = Colors::CLEAR, "scale"_a = 1.0, "ns"_a = "Mesh",
           "id"_a = 0)
      .def("publish_text",
           py::overload_cast<const geometry_msgs::msg::Pose&, const std::string&, Colors, Scales,
                             bool>(&RvizVisualTools::publishText),
           "pose"_a, "text"_a, "color"_a = Colors::WHITE, "scale"_a = Scales::MEDIUM,
           "static_id"_a = true);
}
}  // namespace rviz_visual_tools
