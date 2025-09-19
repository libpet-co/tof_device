#ifndef TOFSENSEINIT_H
#define TOFSENSEINIT_H

#include <map>
#include <unordered_map>

#include <nlink_parser/msg/tofsense_cascade.hpp>
#include <nlink_parser/msg/tofsense_frame0.hpp>
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>

#include "protocol_extracter/nprotocol_extracter.h"

namespace tofsense {
class Init {
public:
  Init(const rclcpp::Node::SharedPtr &node, NProtocolExtracter *protocol_extraction,
       serial::Serial *serial);

private:
  void InitFrame0(NProtocolExtracter *protocol_extraction);

  std::unordered_map<NProtocolBase *, rclcpp::PublisherBase::SharedPtr> publishers_;

  std::map<int, nlink_parser::msg::TofsenseFrame0> frame0_map_;

  serial::Serial *serial_;
  rclcpp::Node::WeakPtr node_;

  const int frequency_ = 10;
  bool is_inquire_mode_ = true;

  rclcpp::TimerBase::SharedPtr timer_scan_;
  rclcpp::TimerBase::SharedPtr timer_read_;
  bool timer_read_active_{false};
  uint8_t node_index_ = 0;
};

} // namespace tofsense
#endif // TOFSENSEINIT_H
