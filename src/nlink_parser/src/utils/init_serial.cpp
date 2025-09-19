#include "init_serial.h"

#include <stdexcept>
#include <string>

void initSerial(const rclcpp::Node::SharedPtr &node, serial::Serial *serial) {
  if (!serial) {
    throw std::invalid_argument("serial pointer must not be null");
  }
  try {
    const auto port_name =
        node->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
    const auto baud_rate = node->declare_parameter<int>("baud_rate", 921600);

    serial->setPort(port_name);
    serial->setBaudrate(static_cast<uint32_t>(baud_rate));
    RCLCPP_INFO(node->get_logger(), "try to open serial port with %s,%d",
                port_name.c_str(), baud_rate);
    auto timeout = serial::Timeout::simpleTimeout(10);
    serial->setTimeout(timeout);
    serial->open();

    if (serial->isOpen()) {
      RCLCPP_INFO(node->get_logger(),
                  "Serial port opened successfully, waiting for data.");
    } else {
      throw std::runtime_error(
          "Failed to open serial port, please check and retry.");
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Unhandled Exception: %s", e.what());
    throw;
  }
}
