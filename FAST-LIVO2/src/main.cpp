#include "LIVMapper.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);

  try
  {
    rclcpp::Node::SharedPtr node;
    LIVMapper mapper(node, "laserMapping", options);
    image_transport::ImageTransport transport(mapper.node);
    mapper.initializeSubscribersAndPublishers(node, transport);
    mapper.run(node);
  }
  catch (const std::exception &error)
  {
    RCLCPP_ERROR(rclcpp::get_logger("fast_livo"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
