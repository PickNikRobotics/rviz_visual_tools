#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <rclcpp/serialization.hpp>

namespace py = pybind11;
using py::literals::operator""_a;

PYBIND11_EXPORT py::object createMessage(const std::string& ros_msg_name)
{
  // find delimiting '/' in ros msg name
  std::size_t pos = ros_msg_name.find('/');
  // import module
  py::module m = py::module::import((ros_msg_name.substr(0, pos) + ".msg").c_str());
  // retrieve type instance
  py::object cls = m.attr(ros_msg_name.substr(pos + 1).c_str());
  // create message instance
  return cls();
}

PYBIND11_EXPORT bool convertible(const pybind11::handle& h, const std::string& ros_msg_name)
{
  PyObject* o = h.attr("__class__").attr("__name__").ptr();
  std::size_t pos = ros_msg_name.find_last_of('/');
  std::string class_name = ros_msg_name.substr(pos + 1);
  return py::cast<std::string>(o) == class_name;
}

namespace pybind11
{
namespace detail
{
// Base class for type conversion (C++ <-> python) of ROS message types
template <typename T>
struct RosMsgTypeCaster
{
  // C++ -> Python
  static handle cast(const T& src, return_value_policy /* policy */, handle /* parent */)
  {
    // serialize src
    rclcpp::Serialization<T> serializer;
    rclcpp::SerializedMessage serialized_msg;
    serializer.serialize_message(&src, &serialized_msg);
    py::bytes bytes =
        py::bytes(reinterpret_cast<const char*>(serialized_msg.get_rcl_serialized_message().buffer),
                  serialized_msg.get_rcl_serialized_message().buffer_length);

    // get Python object type
    const std::string ros_msg_name = rosidl_generator_traits::name<T>();

    // find delimiting '/' in ros_msg_name
    std::size_t pos1 = ros_msg_name.find('/');
    std::size_t pos2 = ros_msg_name.find('/', pos1 + 1);
    py::module m = py::module::import((ros_msg_name.substr(0, pos1) + ".msg").c_str());

    // retrieve type instance
    py::object cls = m.attr(ros_msg_name.substr(pos2 + 1).c_str());

    // deserialize into python object
    py::module rclpy = py::module::import("rclpy.serialization");
    py::object msg = rclpy.attr("deserialize_message")(bytes, cls);

    return msg.release();
  }

  // Python -> C++
  bool load(handle src, bool /*convert*/)
  {
    // check datatype of src
    if (!convertible(src, rosidl_generator_traits::name<T>()))
      return false;

    // serialize src into python buffer
    py::module rclpy = py::module::import("rclpy.serialization");
    py::bytes bytes = rclpy.attr("serialize_message")(src);

    // deserialize into C++ object
    rcl_serialized_message_t rcl_serialized_msg = rmw_get_zero_initialized_serialized_message();
    char* serialized_buffer;
    Py_ssize_t length;
    if (PYBIND11_BYTES_AS_STRING_AND_SIZE(bytes.ptr(), &serialized_buffer, &length))
    {
      throw py::error_already_set();
    }
    if (length < 0)
    {
      throw py::error_already_set();
    }
    rcl_serialized_msg.buffer_capacity = length;
    rcl_serialized_msg.buffer_length = length;
    rcl_serialized_msg.buffer = reinterpret_cast<uint8_t*>(serialized_buffer);
    rmw_ret_t rmw_ret = rmw_deserialize(
        &rcl_serialized_msg, rosidl_typesupport_cpp::get_message_type_support_handle<T>(), &value);
    if (RMW_RET_OK != rmw_ret)
    {
      throw std::runtime_error("failed to deserialize ROS message");
    }
    return true;
  }

  PYBIND11_TYPE_CASTER(T, _<T>());
};

template <typename T>
struct type_caster<T, enable_if_t<rosidl_generator_traits::is_message<T>::value>>
  : RosMsgTypeCaster<T>
{
};
}  // namespace detail
}  // namespace pybind11

namespace rviz_visual_tools
{

// The idea was taken from
// https://github.com/RobotLocomotion/drake-ros/blob/main/drake_ros/core/drake_ros.h
/** A ROS interface that wraps a live rclcpp node.*/
class RvizVisualToolsNode final
{
public:
  RvizVisualToolsNode(const std::string& node_name)
  {
    node = rclcpp::Node::make_unique(node_name);
    executor.add_node(node->get_node_base_interface());
  }

  ~RvizVisualToolsNode()
  {
    executor.remove_node(node->get_node_base_interface());
    stop_spin_thread();
  }

  [[nodiscard]] const rclcpp::Node& getNode() const
  {
    return *node;
  }

  [[nodiscard]] rclcpp::Node* getMutableNode() const
  {
    return node.get();
  }

  void spin_all(int timeout_millis = 0)
  {
    executor.spin_all(std::chrono::milliseconds(timeout_millis));
  }

  void start_spin_thread()
  {
    stop_spin_thread();
    executor_thread = std::thread([this] { executor.spin(); });
  }

  void stop_spin_thread()
  {
    executor.cancel();
    if (executor_thread.joinable())
    {
      executor_thread.join();
    }
  }

private:
  rclcpp::Node::UniquePtr node;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread executor_thread;
};

PYBIND11_MODULE(pyrviz_visual_tools, m)
{
  py::class_<RvizVisualToolsNode>(m, "RvizVisualToolsNode")
      .def(py::init<const std::string&>(), "node_name"_a)
      .def("spin_all", &RvizVisualToolsNode::spin_all, "timeout_millis"_a = 0)
      .def("start_spin_thread", &RvizVisualToolsNode::start_spin_thread)
      .def("stop_spin_thread", &RvizVisualToolsNode::stop_spin_thread);

  /**Initialize ROS 2's global context.
   This function decorates a `rclcpp::init` invocation.
  */
  m.def(
      "init",
      [](std::vector<std::string> args) {
        std::vector<const char*> raw_args;
        raw_args.reserve(args.size());
        for (auto& arg : args)
        {
          raw_args.push_back(arg.c_str());
        }
        if (!rclcpp::ok())
        {
          rclcpp::init(raw_args.size(), raw_args.data());
        }
      },
      py::arg("args") = std::vector<std::string>{});
  m.def("shutdown", &rclcpp::shutdown);

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
      .def(py::init([](RvizVisualToolsNode* node, const std::string& base_frame,
                       const std::string& marker_topic) {
        return RvizVisualTools(base_frame, marker_topic, node->getMutableNode());
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
      .def("get_random_color", &RvizVisualTools::getRandColor)
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
