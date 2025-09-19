#include "init.h"

#include <chrono>

#include "nlink_protocol.h"
#include "nlink_unpack/nlink_tofsense_frame0.h"
#include "nlink_unpack/nlink_utils.h"
#include "nutils.h"

namespace {
class NTS_ProtocolFrame0 : public NLinkProtocol {
public:
  NTS_ProtocolFrame0()
      : NLinkProtocol(true, g_nts_frame0.fixed_part_size,
                      {g_nts_frame0.frame_header, g_nts_frame0.function_mark}) {}

protected:
  void UnpackFrameData(const uint8_t *data) override {
    g_nts_frame0.UnpackData(data, length());
  }
};

#pragma pack(push, 1)
struct CommandRead {
  char header[2]{0x57, 0x10};
  uint8_t reserved0[2]{0xff, 0xff};
  uint8_t id{};
  uint8_t reserved1[2]{0xff, 0xff};
  uint8_t checkSum{};
};
#pragma pack(pop)
} // namespace

namespace tofsense {
using nlink_parser::msg::TofsenseCascade;
using nlink_parser::msg::TofsenseFrame0;

TofsenseFrame0 g_msg_frame0;

Init::Init(const rclcpp::Node::SharedPtr &node, NProtocolExtracter *protocol_extraction,
           serial::Serial *serial)
    : serial_(serial), node_(node) {
  if (serial_) {
    is_inquire_mode_ = node->declare_parameter<bool>("inquire_mode", true);
  } else {
    is_inquire_mode_ = false;
  }

  InitFrame0(protocol_extraction);
}

void Init::InitFrame0(NProtocolExtracter *protocol_extraction) {
  static auto protocol_frame0_ = new NTS_ProtocolFrame0;
  protocol_extraction->AddProtocol(protocol_frame0_);
  protocol_frame0_->SetHandleDataCallback([this, protocol_frame0_] {
    auto node = node_.lock();
    if (!node) {
      return;
    }
    if (!publishers_[protocol_frame0_]) {
      std::string topic;
      if (is_inquire_mode_) {
        topic = "nlink_tofsense_cascade";
        publishers_[protocol_frame0_] = node->create_publisher<TofsenseCascade>(
            topic, rclcpp::QoS(rclcpp::KeepLast(50)));
      } else {
        topic = "nlink_tofsense_frame0";
        publishers_[protocol_frame0_] = node->create_publisher<TofsenseFrame0>(
            topic, rclcpp::QoS(rclcpp::KeepLast(50)));
      }
      TopicAdvertisedTip(node->get_logger(), topic.c_str());
    }

    const auto &data = g_nts_frame0.result;

    g_msg_frame0.id = data.id;
    g_msg_frame0.system_time = data.system_time;
    g_msg_frame0.dis = data.dis;
    g_msg_frame0.dis_status = data.dis_status;
    g_msg_frame0.signal_strength = data.signal_strength;
    g_msg_frame0.range_precision = data.range_precision;

    if (is_inquire_mode_) {
      frame0_map_[data.id] = g_msg_frame0;
    } else {
      auto iter = publishers_.find(protocol_frame0_);
      if (iter != publishers_.end()) {
        auto publisher = std::static_pointer_cast<rclcpp::Publisher<TofsenseFrame0>>(iter->second);
        if (publisher) {
          publisher->publish(g_msg_frame0);
        }
      }
    }
  });

  if (is_inquire_mode_ && serial_) {
    auto node = node_.lock();
    if (!node) {
      return;
    }
    timer_scan_ = node->create_wall_timer(
        std::chrono::duration<double>(1.0 / frequency_),
        [this, protocol_frame0_]() {
          frame0_map_.clear();
          node_index_ = 0;
          timer_read_active_ = true;
          if (timer_read_) {
            timer_read_->reset();
          }
        });
    timer_read_ = node->create_wall_timer(
        std::chrono::milliseconds(6),
        [this, protocol_frame0_]() {
          if (!timer_read_active_) {
            return;
          }
          if (node_index_ >= 8) {
            if (!frame0_map_.empty()) {
              TofsenseCascade msg_cascade;
              for (const auto &msg : frame0_map_) {
                msg_cascade.nodes.push_back(msg.second);
              }
              auto iter = publishers_.find(protocol_frame0_);
              if (iter != publishers_.end()) {
                auto publisher = std::static_pointer_cast<rclcpp::Publisher<TofsenseCascade>>(iter->second);
                if (publisher) {
                  publisher->publish(msg_cascade);
                }
              }
            }
            timer_read_active_ = false;
            if (timer_read_) {
              timer_read_->cancel();
            }
          } else {
            CommandRead command{};
            command.id = node_index_;
            auto data = reinterpret_cast<uint8_t *>(&command);
            NLink_UpdateCheckSum(data, sizeof(CommandRead));
            if (serial_) {
              serial_->write(data, sizeof(CommandRead));
            }
            ++node_index_;
          }
        });
    timer_read_->cancel();
  }
}

} // namespace tofsense
