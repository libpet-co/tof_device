#ifndef IOT_INIT_H
#define IOT_INIT_H

#include <unordered_map>

#include <nlink_parser/msg/iot_frame0.hpp>
#include <rclcpp/rclcpp.hpp>

#include "protocol_extracter/nprotocol_extracter.h"

namespace iot {
class Init {
public:
  explicit Init(const rclcpp::Node::SharedPtr &node, NProtocolExtracter *protocol_extraction);

private:
  void InitFrame0(NProtocolExtracter *protocol_extraction);
  std::unordered_map<NProtocolBase *, rclcpp::PublisherBase::SharedPtr> publishers_;
  rclcpp::Node::WeakPtr node_;
};

} // namespace iot
#endif // IOT_INIT_H
