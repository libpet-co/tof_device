#include "init.h"

#include <cstring>
#include <stdexcept>

#include "../linktrack/protocols.h"
#include "nlink_parser/msg/linktrack_aoa_nodeframe0.hpp"
#include "nlink_parser/msg/linktrack_nodeframe0.hpp"
#include "nlink_protocol.h"
#include "nlink_unpack/nlink_linktrack_aoa_nodeframe0.h"
#include "nlink_unpack/nlink_linktrack_nodeframe0.h"
#include "nutils.h"
#include "std_msgs/msg/string.hpp"

namespace linktrack_aoa {

using nlink_parser::msg::LinktrackAoaNodeframe0;
using nlink_parser::msg::LinktrackNodeframe0;
using std_msgs::msg::String;

LinktrackNodeframe0 g_msg_nodeframe0;
LinktrackAoaNodeframe0 g_msg_aoa_nodeframe0;

Init::Init(const rclcpp::Node::SharedPtr &node, NProtocolExtracter *protocol_extraction,
           serial::Serial *serial)
    : node_(node), serial_(serial) {
  initDataTransmission();
  initNodeFrame0(protocol_extraction);
  InitAoaNodeFrame0(protocol_extraction);
}

void Init::initDataTransmission() {
  if (!serial_) {
    return;
  }
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Node expired before setting up data transmission");
  }
  dt_sub_ = node->create_subscription<String>(
      "nlink_linktrack_data_transmission", rclcpp::QoS(1000),
      [this](const String::SharedPtr msg) {
        if (serial_) {
          serial_->write(msg->data);
        }
      });
}

void Init::initNodeFrame0(NProtocolExtracter *protocol_extraction) {
  auto protocol = new NLT_ProtocolNodeFrame0;
  protocol_extraction->AddProtocol(protocol);
  protocol->SetHandleDataCallback([this, protocol] {
    auto node = node_.lock();
    if (!node) {
      return;
    }
    if (!publishers_[protocol]) {
      const auto topic = "nlink_linktrack_nodeframe0";
      publishers_[protocol] = node->create_publisher<LinktrackNodeframe0>(
          topic, rclcpp::QoS(rclcpp::KeepLast(200)));
      TopicAdvertisedTip(node->get_logger(), topic.c_str());
    }
    const auto &data = g_nlt_nodeframe0.result;
    auto &msg_data = g_msg_nodeframe0;
    auto &msg_nodes = msg_data.nodes;

    msg_data.role = data.role;
    msg_data.id = data.id;

    msg_nodes.resize(data.valid_node_count);
    for (size_t i = 0; i < data.valid_node_count; ++i) {
      auto &msg_node = msg_nodes[i];
      auto node_data = data.nodes[i];
      msg_node.id = node_data->id;
      msg_node.role = node_data->role;
      msg_node.data.resize(node_data->data_length);
      std::memcpy(msg_node.data.data(), node_data->data, node_data->data_length);
    }

    auto iter = publishers_.find(protocol);
    if (iter != publishers_.end()) {
      auto publisher = std::static_pointer_cast<rclcpp::Publisher<LinktrackNodeframe0>>(iter->second);
      if (publisher) {
        publisher->publish(msg_data);
      }
    }
  });
}

void Init::InitAoaNodeFrame0(NProtocolExtracter *protocol_extraction) {
  auto protocol = new NLTAoa_ProtocolNodeFrame0;
  protocol_extraction->AddProtocol(protocol);
  protocol->SetHandleDataCallback([this, protocol] {
    auto node = node_.lock();
    if (!node) {
      return;
    }
    if (!publishers_[protocol]) {
      const auto topic = "nlink_linktrack_aoa_nodeframe0";
      publishers_[protocol] = node->create_publisher<LinktrackAoaNodeframe0>(
          topic, rclcpp::QoS(rclcpp::KeepLast(200)));
      TopicAdvertisedTip(node->get_logger(), topic.c_str());
    }
    const auto &data = g_nltaoa_nodeframe0.result;
    auto &msg_data = g_msg_aoa_nodeframe0;
    auto &msg_nodes = msg_data.nodes;

    msg_data.role = data.role;
    msg_data.id = data.id;
    msg_data.local_time = data.local_time;
    msg_data.system_time = data.system_time;
    msg_data.voltage = data.voltage;

    msg_nodes.resize(data.valid_node_count);
    for (size_t i = 0; i < data.valid_node_count; ++i) {
      auto &msg_node = msg_nodes[i];
      auto node_data = data.nodes[i];
      msg_node.id = node_data->id;
      msg_node.role = node_data->role;
      msg_node.dis = node_data->dis;
      msg_node.angle = node_data->angle;
      msg_node.fp_rssi = node_data->fp_rssi;
      msg_node.rx_rssi = node_data->rx_rssi;
    }

    auto iter = publishers_.find(protocol);
    if (iter != publishers_.end()) {
      auto publisher = std::static_pointer_cast<rclcpp::Publisher<LinktrackAoaNodeframe0>>(iter->second);
      if (publisher) {
        publisher->publish(msg_data);
      }
    }
  });
}

} // namespace linktrack_aoa
