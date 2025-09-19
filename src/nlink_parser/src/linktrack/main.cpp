#include <rclcpp/rclcpp.hpp>

#include "init.h"
#include "init_serial.h"
#include "protocol_extracter/nprotocol_extracter.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>

void printHexData(const std::string &data) {
  if (!data.empty()) {
    std::cout << "data received: ";
    for (size_t i = 0; i < data.size(); ++i) {
      std::cout << std::hex << std::setfill('0') << std::setw(2)
                << static_cast<int>(static_cast<uint8_t>(data.at(i))) << " ";
    }
    std::cout << std::dec << std::endl;
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("linktrack_parser");
  serial::Serial serial;
  try {
    initSerial(node, &serial);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(node->get_logger(), "Failed to initialise serial port: %s", e.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  NProtocolExtracter protocol_extraction;
  linktrack::Init init(node, &protocol_extraction, &serial);

  rclcpp::WallRate loop_rate(std::chrono::milliseconds(1));
  while (rclcpp::ok()) {
    auto available_bytes = serial.available();
    std::string str_received;
    if (available_bytes) {
      serial.read(str_received, available_bytes);
      // printHexData(str_received);
      protocol_extraction.AddNewData(str_received);
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
