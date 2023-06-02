#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rviz_visual_tools/binding_utils.hpp>

namespace py = pybind11;

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

rviz_visual_tools::RvizVisualToolsNode::RvizVisualToolsNode(const std::string& node_name)
  : Node(node_name)
{
  executor.add_node(get_node_base_interface());
}

rviz_visual_tools::RvizVisualToolsNode::~RvizVisualToolsNode()
{
  executor.remove_node(get_node_base_interface());
  stop_spin_thread();
}

void rviz_visual_tools::RvizVisualToolsNode::spin_all(int timeout_millis)
{
  executor.spin_all(std::chrono::milliseconds(timeout_millis));
}

void rviz_visual_tools::RvizVisualToolsNode::start_spin_thread()
{
  stop_spin_thread();
  executor_thread = std::thread([this] { executor.spin(); });
}

void rviz_visual_tools::RvizVisualToolsNode::stop_spin_thread()
{
  executor.cancel();
  if (executor_thread.joinable())
  {
    executor_thread.join();
  }
}
