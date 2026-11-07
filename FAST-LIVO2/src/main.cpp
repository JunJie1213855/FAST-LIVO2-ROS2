#include "LIVMapper.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);

  rclcpp::Node::SharedPtr nh;
  image_transport::ImageTransport it_(nh);
  LIVMapper mapper(nh, "laserMapping", options);
  mapper.initializeSubscribersAndPublishers(nh, it_);

  // 循环运行
  // 数据同步 -> imu 前向传播 -> 点云去畸变 -> lio 状态更新 -> 图像块匹配 -> vio 状态更新 -> 地图更新
  mapper.run(nh);
  rclcpp::shutdown();
  return 0;
}
