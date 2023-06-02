#pragma once

#include <pybind11/pybind11.h>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosidl_runtime_cpp/traits.hpp>

PYBIND11_EXPORT pybind11::object createMessage(const std::string& ros_msg_name);

PYBIND11_EXPORT bool convertible(const pybind11::handle& h, const std::string& ros_msg_name);

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
    pybind11::bytes bytes = pybind11::bytes(
        reinterpret_cast<const char*>(serialized_msg.get_rcl_serialized_message().buffer),
        serialized_msg.get_rcl_serialized_message().buffer_length);

    // get Python object type
    const std::string ros_msg_name = rosidl_generator_traits::name<T>();

    // find delimiting '/' in ros_msg_name
    std::size_t pos1 = ros_msg_name.find('/');
    std::size_t pos2 = ros_msg_name.find('/', pos1 + 1);
    pybind11::module m = pybind11::module::import((ros_msg_name.substr(0, pos1) + ".msg").c_str());

    // retrieve type instance
    pybind11::object cls = m.attr(ros_msg_name.substr(pos2 + 1).c_str());

    // deserialize into python object
    pybind11::module rclpy = pybind11::module::import("rclpy.serialization");
    pybind11::object msg = rclpy.attr("deserialize_message")(bytes, cls);

    return msg.release();
  }

  // Python -> C++
  bool load(handle src, bool /*convert*/)
  {
    // check datatype of src
    if (!convertible(src, rosidl_generator_traits::name<T>()))
      return false;

    // serialize src into python buffer
    pybind11::module rclpy = pybind11::module::import("rclpy.serialization");
    pybind11::bytes bytes = rclpy.attr("serialize_message")(src);

    // deserialize into C++ object
    rcl_serialized_message_t rcl_serialized_msg = rmw_get_zero_initialized_serialized_message();
    char* serialized_buffer;
    Py_ssize_t length;
    if (PYBIND11_BYTES_AS_STRING_AND_SIZE(bytes.ptr(), &serialized_buffer, &length))
    {
      throw pybind11::error_already_set();
    }
    if (length < 0)
    {
      throw pybind11::error_already_set();
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
class RvizVisualToolsNode final : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(RvizVisualToolsNode)

  RvizVisualToolsNode(const std::string& node_name);

  ~RvizVisualToolsNode();

  void spin_all(int timeout_millis = 0);

  void start_spin_thread();

  void stop_spin_thread();

private:
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread executor_thread;
};
}  // namespace rviz_visual_tools
