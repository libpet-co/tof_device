#ifndef LINKTRACKAOAINIT_H
#define LINKTRACKAOAINIT_H

#include <memory>
#include <unordered_map>

#include <nlink_parser/msg/linktrack_aoa_nodeframe0.hpp>
#include <nlink_parser/msg/linktrack_nodeframe0.hpp>
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>

#include "protocol_extracter/nprotocol_extracter.h"

namespace linktrack_aoa {
class Init {
public:
  Init(const rclcpp::Node::SharedPtr &node, NProtocolExtracter *protocol_extraction,
       serial::Serial *serial);

private:
  void initDataTransmission();
  void initNodeFrame0(NProtocolExtracter *protocol_extraction);
  void InitAoaNodeFrame0(NProtocolExtracter *protocol_extraction);
  std::unordered_map<NProtocolBase *, rclcpp::PublisherBase::SharedPtr> publishers_;
  rclcpp::Node::WeakPtr node_;
  serial::Serial *serial_;
  rclcpp::SubscriptionBase::SharedPtr dt_sub_;
};
} // namespace linktrack_aoa

#endif // LINKTRACKAOAINIT_H
