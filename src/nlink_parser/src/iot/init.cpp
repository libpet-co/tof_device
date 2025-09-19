#include "init.h"

#include "nlink_protocol.h"
#include "nlink_unpack/nlink_iot_frame0.h"
#include "nlink_unpack/nlink_utils.h"
#include "nutils.h"
#include <cstring>

namespace {
class ProtocolFrame0 : public NLinkProtocolVLength {
public:
  ProtocolFrame0()
      : NLinkProtocolVLength(
            true, g_iot_frame0.fixed_part_size,
            {g_iot_frame0.frame_header, g_iot_frame0.function_mark}) {}

protected:
  void UnpackFrameData(const uint8_t *data) override {
    g_iot_frame0.UnpackData(data, length());
  }
};

} // namespace

namespace iot {
using nlink_parser::msg::IotFrame0;
using nlink_parser::msg::IotFrame0Node;

IotFrame0 g_msg_iotframe0;

Init::Init(const rclcpp::Node::SharedPtr &node, NProtocolExtracter *protocol_extraction)
    : node_(node) {
  InitFrame0(protocol_extraction);
}

void Init::InitFrame0(NProtocolExtracter *protocol_extraction) {
  static auto protocol = new ProtocolFrame0;
  protocol_extraction->AddProtocol(protocol);
  protocol->SetHandleDataCallback([this, protocol] {
    auto node = node_.lock();
    if (!node) {
      return;
    }
    if (!publishers_[protocol]) {
      const std::string topic = "nlink_iot_frame0";
      publishers_[protocol] = node->create_publisher<IotFrame0>(
          topic, rclcpp::QoS(rclcpp::KeepLast(50)));
      TopicAdvertisedTip(node->get_logger(), topic.c_str());
    }

    const auto &data = g_iot_frame0;
    g_msg_iotframe0.uid = data.uid;
    g_msg_iotframe0.system_time = data.system_time;
    g_msg_iotframe0.io_status = *(const uint8_t *)&(data.io_status);
    g_msg_iotframe0.nodes.resize(data.node_count);
    for (int i = 0; i < data.node_count; ++i) {
      auto &dst = g_msg_iotframe0.nodes[i];
      const auto &src = data.nodes[i];
      dst.uid = src.uid;
      dst.dis = src.dis;
      dst.aoa_angle_horizontal = src.aoa_angle_horizontal;
      dst.aoa_angle_vertical = src.aoa_angle_vertical;
      dst.fp_rssi = src.fp_rssi;
      dst.rx_rssi = src.rx_rssi;
      dst.user_data.resize(src.user_data_len);
      std::memcpy(dst.user_data.data(), src.user_data, src.user_data_len);
    }

    auto iter = publishers_.find(protocol);
    if (iter != publishers_.end()) {
      auto publisher = std::static_pointer_cast<rclcpp::Publisher<IotFrame0>>(iter->second);
      if (publisher) {
        publisher->publish(g_msg_iotframe0);
      }
    }
  });
}

} // namespace iot
