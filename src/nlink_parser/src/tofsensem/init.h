#ifndef TOFSENSEMINIT_H
#define TOFSENSEMINIT_H

#include <unordered_map>

#include <nlink_parser/msg/tofsense_m_frame0.hpp>
#include <rclcpp/rclcpp.hpp>

#include "protocol_extracter/nprotocol_extracter.h"

namespace tofsensem {
class Init {
public:
  Init(const rclcpp::Node::SharedPtr &node, NProtocolExtracter *protocol_extraction);

private:
  void InitFrame0(NProtocolExtracter *protocol_extraction);
  std::unordered_map<NProtocolBase *, rclcpp::PublisherBase::SharedPtr> publishers_;
  rclcpp::Node::WeakPtr node_;
};

} // namespace tofsensem
#endif // TOFSENSEMINIT_H
