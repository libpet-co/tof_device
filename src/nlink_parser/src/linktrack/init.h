#ifndef LINKTRACKINIT_H
#define LINKTRACKINIT_H

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>

#include <memory>
#include <unordered_map>

#include "nlink_unpack/nlink_utils.h"
#include "protocol_extracter/nprotocol_extracter.h"

class NProtocolExtracter;
namespace linktrack {
class Init {
public:
  Init(const rclcpp::Node::SharedPtr &node, NProtocolExtracter *protocol_extraction,
       serial::Serial *serial);

private:
  void initDataTransmission();
  void initAnchorFrame0(NProtocolExtracter *protocol_extraction);
  void initTagFrame0(NProtocolExtracter *protocol_extraction);
  void initNodeFrame0(NProtocolExtracter *protocol_extraction);
  void initNodeFrame1(NProtocolExtracter *protocol_extraction);
  void initNodeFrame2(NProtocolExtracter *protocol_extraction);
  void initNodeFrame3(NProtocolExtracter *protocol_extraction);
  void initNodeFrame4(NProtocolExtracter *protocol_extraction);
  void initNodeFrame5(NProtocolExtracter *protocol_extraction);
  void initNodeFrame6(NProtocolExtracter *protocol_extraction);

  std::unordered_map<NProtocolBase *, rclcpp::PublisherBase::SharedPtr> publishers_;
  rclcpp::Node::WeakPtr node_;
  serial::Serial *serial_;
  rclcpp::SubscriptionBase::SharedPtr dt_sub_;
};
} // namespace linktrack

#endif // LINKTRACKINIT_H
