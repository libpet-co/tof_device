#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "init.h"
#include "init_serial.h"
#include "protocol_extracter/nprotocol_extracter.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("iot_parser");
  serial::Serial serial;
  try {
    initSerial(node, &serial);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(node->get_logger(), "Failed to initialise serial port: %s", e.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  NProtocolExtracter extracter;
  iot::Init init(node, &extracter);
  rclcpp::WallRate loop_rate(std::chrono::milliseconds(1));
  while (rclcpp::ok()) {
    auto available_bytes = serial.available();
    std::string str_received;
    if (available_bytes) {
      serial.read(str_received, available_bytes);
      extracter.AddNewData(str_received);
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
