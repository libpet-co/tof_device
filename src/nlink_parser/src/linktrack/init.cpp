#include "init.h"

#include <cstring>
#include <stdexcept>

#include "nlink_parser/msg/linktrack_anchorframe0.hpp"
#include "nlink_parser/msg/linktrack_nodeframe0.hpp"
#include "nlink_parser/msg/linktrack_nodeframe1.hpp"
#include "nlink_parser/msg/linktrack_nodeframe2.hpp"
#include "nlink_parser/msg/linktrack_nodeframe3.hpp"
#include "nlink_parser/msg/linktrack_nodeframe4.hpp"
#include "nlink_parser/msg/linktrack_nodeframe5.hpp"
#include "nlink_parser/msg/linktrack_nodeframe6.hpp"
#include "nlink_parser/msg/linktrack_tagframe0.hpp"
#include "std_msgs/msg/string.hpp"

#include "nutils.h"
#include "protocols.h"

#define ARRAY_ASSIGN(DEST, SRC)                                                \
  for (size_t _CNT = 0; _CNT < sizeof(SRC) / sizeof(SRC[0]); ++_CNT) {         \
    DEST[_CNT] = SRC[_CNT];                                                    \
  }

namespace linktrack {

using nlink_parser::msg::LinktrackAnchorframe0;
using nlink_parser::msg::LinktrackNodeframe0;
using nlink_parser::msg::LinktrackNodeframe1;
using nlink_parser::msg::LinktrackNodeframe2;
using nlink_parser::msg::LinktrackNodeframe3;
using nlink_parser::msg::LinktrackNodeframe4;
using nlink_parser::msg::LinktrackNodeframe5;
using nlink_parser::msg::LinktrackNodeframe6;
using nlink_parser::msg::LinktrackTagframe0;
using std_msgs::msg::String;

LinktrackAnchorframe0 g_msg_anchorframe0;
LinktrackTagframe0 g_msg_tagframe0;
LinktrackNodeframe0 g_msg_nodeframe0;
LinktrackNodeframe1 g_msg_nodeframe1;
LinktrackNodeframe2 g_msg_nodeframe2;
LinktrackNodeframe3 g_msg_nodeframe3;
LinktrackNodeframe4 g_msg_nodeframe4;
LinktrackNodeframe5 g_msg_nodeframe5;
LinktrackNodeframe6 g_msg_nodeframe6;

Init::Init(const rclcpp::Node::SharedPtr &node,
           NProtocolExtracter *protocol_extraction, serial::Serial *serial)
    : node_(node), serial_(serial) {
  initDataTransmission();
  initAnchorFrame0(protocol_extraction);
  initTagFrame0(protocol_extraction);
  initNodeFrame0(protocol_extraction);
  initNodeFrame1(protocol_extraction);
  initNodeFrame2(protocol_extraction);
  initNodeFrame3(protocol_extraction);
  initNodeFrame4(protocol_extraction);
  initNodeFrame5(protocol_extraction);
  initNodeFrame6(protocol_extraction);
}

void Init::initDataTransmission() {
  if (!serial_) {
    return;
  }
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Node expired before initializing data transmission");
  }
  dt_sub_ = node->create_subscription<String>(
      "nlink_linktrack_data_transmission", rclcpp::QoS(1000),
      [this](const String::SharedPtr msg) {
        if (serial_) {
          serial_->write(msg->data);
        }
      });
}

void Init::initAnchorFrame0(NProtocolExtracter *protocol_extraction) {
  auto protocol = new NLT_ProtocolAnchorFrame0;
  protocol_extraction->AddProtocol(protocol);
  protocol->SetHandleDataCallback([this, protocol] {
    auto node = node_.lock();
    if (!node) {
      return;
    }
    if (!publishers_[protocol]) {
      const auto topic = "nlink_linktrack_anchorframe0";
      auto publisher = node->create_publisher<LinktrackAnchorframe0>(
          topic, rclcpp::QoS(rclcpp::KeepLast(200)));
      TopicAdvertisedTip(node->get_logger(), topic);
      publishers_[protocol] = publisher;
    }
    auto publisher = std::static_pointer_cast<rclcpp::Publisher<LinktrackAnchorframe0>>(
        publishers_.at(protocol));
    const auto &data = nlt_anchorframe0_.result;
    g_msg_anchorframe0.role = data.role;
    g_msg_anchorframe0.id = data.id;
    g_msg_anchorframe0.voltage = data.voltage;
    g_msg_anchorframe0.local_time = data.local_time;
    g_msg_anchorframe0.system_time = data.system_time;
    auto &msg_nodes = g_msg_anchorframe0.nodes;
    msg_nodes.clear();
    decltype(g_msg_anchorframe0.nodes)::value_type msg_node;
    for (size_t i = 0, icount = data.valid_node_count; i < icount; ++i) {
      auto node_data = data.nodes[i];
      msg_node.role = node_data->role;
      msg_node.id = node_data->id;
      ARRAY_ASSIGN(msg_node.pos_3d, node_data->pos_3d);
      ARRAY_ASSIGN(msg_node.dis_arr, node_data->dis_arr);
      msg_nodes.push_back(msg_node);
    }
    publisher->publish(g_msg_anchorframe0);
  });
}

void Init::initTagFrame0(NProtocolExtracter *protocol_extraction) {
  auto protocol = new NLT_ProtocolTagFrame0;
  protocol_extraction->AddProtocol(protocol);
  protocol->SetHandleDataCallback([this, protocol] {
    auto node = node_.lock();
    if (!node) {
      return;
    }
    if (!publishers_[protocol]) {
      const auto topic = "nlink_linktrack_tagframe0";
      auto publisher = node->create_publisher<LinktrackTagframe0>(
          topic, rclcpp::QoS(rclcpp::KeepLast(200)));
      TopicAdvertisedTip(node->get_logger(), topic);
      publishers_[protocol] = publisher;
    }
    const auto &data = g_nlt_tagframe0.result;
    auto &msg_data = g_msg_tagframe0;

    msg_data.role = data.role;
    msg_data.id = data.id;
    msg_data.local_time = data.local_time;
    msg_data.system_time = data.system_time;
    msg_data.voltage = data.voltage;
    ARRAY_ASSIGN(msg_data.pos_3d, data.pos_3d);
    ARRAY_ASSIGN(msg_data.eop_3d, data.eop_3d);
    ARRAY_ASSIGN(msg_data.vel_3d, data.vel_3d);
    ARRAY_ASSIGN(msg_data.dis_arr, data.dis_arr);
    ARRAY_ASSIGN(msg_data.imu_gyro_3d, data.imu_gyro_3d);
    ARRAY_ASSIGN(msg_data.imu_acc_3d, data.imu_acc_3d);
    ARRAY_ASSIGN(msg_data.angle_3d, data.angle_3d);
    ARRAY_ASSIGN(msg_data.quaternion, data.quaternion);

    auto publisher = std::static_pointer_cast<rclcpp::Publisher<LinktrackTagframe0>>(
        publishers_.at(protocol));
    publisher->publish(msg_data);
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
      auto publisher = node->create_publisher<LinktrackNodeframe0>(
          topic, rclcpp::QoS(rclcpp::KeepLast(200)));
      TopicAdvertisedTip(node->get_logger(), topic);
      publishers_[protocol] = publisher;
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

    auto publisher = std::static_pointer_cast<rclcpp::Publisher<LinktrackNodeframe0>>(
        publishers_.at(protocol));
    publisher->publish(msg_data);
  });
}

void Init::initNodeFrame1(NProtocolExtracter *protocol_extraction) {
  auto protocol = new NLT_ProtocolNodeFrame1;
  protocol_extraction->AddProtocol(protocol);
  protocol->SetHandleDataCallback([this, protocol] {
    auto node = node_.lock();
    if (!node) {
      return;
    }
    if (!publishers_[protocol]) {
      const auto topic = "nlink_linktrack_nodeframe1";
      auto publisher = node->create_publisher<LinktrackNodeframe1>(
          topic, rclcpp::QoS(rclcpp::KeepLast(200)));
      TopicAdvertisedTip(node->get_logger(), topic);
      publishers_[protocol] = publisher;
    }
    const auto &data = g_nlt_nodeframe1.result;
    auto &msg_data = g_msg_nodeframe1;
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
      ARRAY_ASSIGN(msg_node.pos_3d, node_data->pos_3d);
    }

    auto publisher = std::static_pointer_cast<rclcpp::Publisher<LinktrackNodeframe1>>(
        publishers_.at(protocol));
    publisher->publish(msg_data);
  });
}

void Init::initNodeFrame2(NProtocolExtracter *protocol_extraction) {
  auto protocol = new NLT_ProtocolNodeFrame2;
  protocol_extraction->AddProtocol(protocol);
  protocol->SetHandleDataCallback([this, protocol] {
    auto node = node_.lock();
    if (!node) {
      return;
    }
    if (!publishers_[protocol]) {
      const auto topic = "nlink_linktrack_nodeframe2";
      auto publisher = node->create_publisher<LinktrackNodeframe2>(
          topic, rclcpp::QoS(rclcpp::KeepLast(200)));
      TopicAdvertisedTip(node->get_logger(), topic);
      publishers_[protocol] = publisher;
    }
    const auto &data = g_nlt_nodeframe2.result;
    auto &msg_data = g_msg_nodeframe2;
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
      ARRAY_ASSIGN(msg_node.pos_3d, node_data->pos_3d);
      ARRAY_ASSIGN(msg_node.quaternion, node_data->quaternion);
    }

    auto publisher = std::static_pointer_cast<rclcpp::Publisher<LinktrackNodeframe2>>(
        publishers_.at(protocol));
    publisher->publish(msg_data);
  });
}

void Init::initNodeFrame3(NProtocolExtracter *protocol_extraction) {
  auto protocol = new NLT_ProtocolNodeFrame3;
  protocol_extraction->AddProtocol(protocol);
  protocol->SetHandleDataCallback([this, protocol] {
    auto node = node_.lock();
    if (!node) {
      return;
    }
    if (!publishers_[protocol]) {
      const auto topic = "nlink_linktrack_nodeframe3";
      auto publisher = node->create_publisher<LinktrackNodeframe3>(
          topic, rclcpp::QoS(rclcpp::KeepLast(200)));
      TopicAdvertisedTip(node->get_logger(), topic);
      publishers_[protocol] = publisher;
    }
    const auto &data = g_nlt_nodeframe3.result;
    auto &msg_data = g_msg_nodeframe3;
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
      ARRAY_ASSIGN(msg_node.pos_3d, node_data->pos_3d);
      msg_node.dis = node_data->dis;
    }

    auto publisher = std::static_pointer_cast<rclcpp::Publisher<LinktrackNodeframe3>>(
        publishers_.at(protocol));
    publisher->publish(msg_data);
  });
}

void Init::initNodeFrame4(NProtocolExtracter *protocol_extraction) {
  auto protocol = new NLT_ProtocolNodeFrame4;
  protocol_extraction->AddProtocol(protocol);
  protocol->SetHandleDataCallback([this, protocol] {
    auto node = node_.lock();
    if (!node) {
      return;
    }
    if (!publishers_[protocol]) {
      const auto topic = "nlink_linktrack_nodeframe4";
      auto publisher = node->create_publisher<LinktrackNodeframe4>(
          topic, rclcpp::QoS(rclcpp::KeepLast(200)));
      TopicAdvertisedTip(node->get_logger(), topic);
      publishers_[protocol] = publisher;
    }
    const auto &data = g_nlt_nodeframe4.result;
    auto &msg_data = g_msg_nodeframe4;
    auto &msg_nodes = msg_data.nodes;

    msg_data.role = data.role;
    msg_data.id = data.id;
    msg_data.system_time = data.system_time;
    msg_data.voltage = data.voltage;

    msg_nodes.resize(data.valid_node_count);
    for (size_t i = 0; i < data.valid_node_count; ++i) {
      auto &msg_node = msg_nodes[i];
      auto node_data = data.nodes[i];
      msg_node.id = node_data->id;
      msg_node.role = node_data->role;
      msg_node.dis = node_data->dis;
      ARRAY_ASSIGN(msg_node.eop_3d, node_data->eop_3d);
    }

    auto publisher = std::static_pointer_cast<rclcpp::Publisher<LinktrackNodeframe4>>(
        publishers_.at(protocol));
    publisher->publish(msg_data);
  });
}

void Init::initNodeFrame5(NProtocolExtracter *protocol_extraction) {
  auto protocol = new NLT_ProtocolNodeFrame5;
  protocol_extraction->AddProtocol(protocol);
  protocol->SetHandleDataCallback([this, protocol] {
    auto node = node_.lock();
    if (!node) {
      return;
    }
    if (!publishers_[protocol]) {
      const auto topic = "nlink_linktrack_nodeframe5";
      auto publisher = node->create_publisher<LinktrackNodeframe5>(
          topic, rclcpp::QoS(rclcpp::KeepLast(200)));
      TopicAdvertisedTip(node->get_logger(), topic);
      publishers_[protocol] = publisher;
    }
    const auto &data = g_nlt_nodeframe5.result;
    auto &msg_data = g_msg_nodeframe5;
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
      msg_node.fp_rssi = node_data->fp_rssi;
      msg_node.rx_rssi = node_data->rx_rssi;
    }

    auto publisher = std::static_pointer_cast<rclcpp::Publisher<LinktrackNodeframe5>>(
        publishers_.at(protocol));
    publisher->publish(msg_data);
  });
}

void Init::initNodeFrame6(NProtocolExtracter *protocol_extraction) {
  auto protocol = new NLT_ProtocolNodeFrame6;
  protocol_extraction->AddProtocol(protocol);
  protocol->SetHandleDataCallback([this, protocol] {
    auto node = node_.lock();
    if (!node) {
      return;
    }
    if (!publishers_[protocol]) {
      const auto topic = "nlink_linktrack_nodeframe6";
      auto publisher = node->create_publisher<LinktrackNodeframe6>(
          topic, rclcpp::QoS(rclcpp::KeepLast(200)));
      TopicAdvertisedTip(node->get_logger(), topic);
      publishers_[protocol] = publisher;
    }
    const auto &data = g_nlt_nodeframe6.result;
    auto &msg_data = g_msg_nodeframe6;
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
      ARRAY_ASSIGN(msg_node.pos_3d, node_data->pos_3d);
      ARRAY_ASSIGN(msg_node.quaternion, node_data->quaternion);
      ARRAY_ASSIGN(msg_node.vel_3d, node_data->vel_3d);
      ARRAY_ASSIGN(msg_node.acc_3d, node_data->acc_3d);
      ARRAY_ASSIGN(msg_node.imu_gyro_3d, node_data->imu_gyro_3d);
      ARRAY_ASSIGN(msg_node.imu_acc_3d, node_data->imu_acc_3d);
    }

    auto publisher = std::static_pointer_cast<rclcpp::Publisher<LinktrackNodeframe6>>(
        publishers_.at(protocol));
    publisher->publish(msg_data);
  });
}

} // namespace linktrack
