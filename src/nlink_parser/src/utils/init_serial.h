#ifndef INITSERIAL_H
#define INITSERIAL_H
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>

void initSerial(const rclcpp::Node::SharedPtr &node, serial::Serial *serial);

#endif // INITSERIAL_H
