// Copyright 2026 Juan Francisco Rascon Crespo

#include <fake_localization/fake_localization.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto return_code{0};

  try
  {
    auto node{std::make_shared<fake_localization::FakeLocalization>()};
    rclcpp::spin(node->get_node_base_interface());
  }
  catch(const std::exception& e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("fake_localization_node"), "%s", e.what());
    return_code = 1;
  }

  rclcpp::shutdown();
  return return_code;
}
