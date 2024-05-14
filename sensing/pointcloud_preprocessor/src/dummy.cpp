#include <rclcpp/rclcpp.hpp>

namespace pointcloud_preprocessor
{
class Dummy : public rclcpp::Node
{
public:
  Dummy(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("pointcloud_preprocessor_filter", options) {}
};
}  // namespace pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::Dummy)
