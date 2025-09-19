#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nlink_parser/msg/linktrack_anchorframe0.hpp>
#include <nlink_parser/msg/linktrack_nodeframe1.hpp>
#include <nlink_parser/msg/linktrack_nodeframe2.hpp>
#include <nlink_parser/msg/linktrack_tagframe0.hpp>
#include <rclcpp/rclcpp.hpp>

#include <map>
#include <sstream>
#include <string>

#include "nutils.h"

namespace {
struct PosePublisher {
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher;
  geometry_msgs::msg::PoseStamped msg;

  void publish() {
    if (publisher) {
      publisher->publish(msg);
    }
  }
};
} // namespace

class LinktrackRvizConverter : public rclcpp::Node {
public:
  LinktrackRvizConverter()
      : rclcpp::Node("linktrack_example"),
        frame_id_(this->declare_parameter<std::string>("map_frame", "linktrack_map")) {
    using std::placeholders::_1;
    anchor_sub_ = this->create_subscription<nlink_parser::msg::LinktrackAnchorframe0>(
        "nlink_linktrack_anchorframe0", rclcpp::QoS(10),
        std::bind(&LinktrackRvizConverter::anchorframe0Callback, this, _1));
    tag_sub_ = this->create_subscription<nlink_parser::msg::LinktrackTagframe0>(
        "nlink_linktrack_tagframe0", rclcpp::QoS(10),
        std::bind(&LinktrackRvizConverter::tagframe0Callback, this, _1));
    nodeframe1_sub_ = this->create_subscription<nlink_parser::msg::LinktrackNodeframe1>(
        "nlink_linktrack_nodeframe1", rclcpp::QoS(10),
        std::bind(&LinktrackRvizConverter::nodeframe1Callback, this, _1));
    nodeframe2_sub_ = this->create_subscription<nlink_parser::msg::LinktrackNodeframe2>(
        "nlink_linktrack_nodeframe2", rclcpp::QoS(10),
        std::bind(&LinktrackRvizConverter::nodeframe2Callback, this, _1));
  }

private:
  void anchorframe0Callback(const nlink_parser::msg::LinktrackAnchorframe0::SharedPtr msg) {
    for (const auto &node : msg->nodes) {
      auto &pose = anchor_poses_[node.id];
      if (!pose.publisher) {
        std::ostringstream string_stream;
        string_stream << "nlt_anchorframe0_pose_node" << static_cast<int>(node.id);
        const auto topic = string_stream.str();
        pose.publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            topic, rclcpp::QoS(10));
        TopicAdvertisedTip(this->get_logger(), topic.c_str());
        pose.msg.header.frame_id = frame_id_;
        pose.msg.pose.orientation.w = 0.0;
        pose.msg.pose.orientation.x = 0.0;
        pose.msg.pose.orientation.y = 0.0;
        pose.msg.pose.orientation.z = 1.0;
      }
      pose.msg.header.stamp = this->now();
      pose.msg.pose.position.x = static_cast<double>(node.pos_3d[0]);
      pose.msg.pose.position.y = static_cast<double>(node.pos_3d[1]);
      pose.msg.pose.position.z = static_cast<double>(node.pos_3d[2]);
      pose.publish();
    }
  }

  void nodeframe1Callback(const nlink_parser::msg::LinktrackNodeframe1::SharedPtr msg) {
    for (const auto &node : msg->nodes) {
      auto &pose = nodeframe1_poses_[node.id];
      if (!pose.publisher) {
        std::ostringstream string_stream;
        string_stream << "nlt_nodeframe1_pose_node" << static_cast<int>(node.id);
        const auto topic = string_stream.str();
        pose.publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            topic, rclcpp::QoS(10));
        TopicAdvertisedTip(this->get_logger(), topic.c_str());
        pose.msg.header.frame_id = frame_id_;
        pose.msg.pose.orientation.w = 0.0;
        pose.msg.pose.orientation.x = 0.0;
        pose.msg.pose.orientation.y = 0.0;
        pose.msg.pose.orientation.z = 1.0;
      }
      pose.msg.header.stamp = this->now();
      pose.msg.pose.position.x = static_cast<double>(node.pos_3d[0]);
      pose.msg.pose.position.y = static_cast<double>(node.pos_3d[1]);
      pose.msg.pose.position.z = static_cast<double>(node.pos_3d[2]);
      pose.publish();
    }
  }

  void tagframe0Callback(const nlink_parser::msg::LinktrackTagframe0::SharedPtr msg) {
    if (!tag_pose_.publisher) {
      const std::string topic = "nlt_tagframe0_pose";
      tag_pose_.publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
          topic, rclcpp::QoS(10));
      TopicAdvertisedTip(this->get_logger(), topic.c_str());
      tag_pose_.msg.header.frame_id = frame_id_;
    }
    tag_pose_.msg.header.stamp = this->now();
    tag_pose_.msg.pose.orientation.w = static_cast<double>(msg->quaternion[0]);
    tag_pose_.msg.pose.orientation.x = static_cast<double>(msg->quaternion[1]);
    tag_pose_.msg.pose.orientation.y = static_cast<double>(msg->quaternion[2]);
    tag_pose_.msg.pose.orientation.z = static_cast<double>(msg->quaternion[3]);
    tag_pose_.msg.pose.position.x = static_cast<double>(msg->pos_3d[0]);
    tag_pose_.msg.pose.position.y = static_cast<double>(msg->pos_3d[1]);
    tag_pose_.msg.pose.position.z = static_cast<double>(msg->pos_3d[2]);
    tag_pose_.publish();
  }

  void nodeframe2Callback(const nlink_parser::msg::LinktrackNodeframe2::SharedPtr msg) {
    if (!nodeframe2_pose_.publisher) {
      const std::string topic = "nlt_nodeframe2_pose";
      nodeframe2_pose_.publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
          topic, rclcpp::QoS(10));
      TopicAdvertisedTip(this->get_logger(), topic.c_str());
      nodeframe2_pose_.msg.header.frame_id = frame_id_;
    }
    nodeframe2_pose_.msg.header.stamp = this->now();
    nodeframe2_pose_.msg.pose.orientation.w = static_cast<double>(msg->quaternion[0]);
    nodeframe2_pose_.msg.pose.orientation.x = static_cast<double>(msg->quaternion[1]);
    nodeframe2_pose_.msg.pose.orientation.y = static_cast<double>(msg->quaternion[2]);
    nodeframe2_pose_.msg.pose.orientation.z = static_cast<double>(msg->quaternion[3]);
    nodeframe2_pose_.msg.pose.position.x = static_cast<double>(msg->pos_3d[0]);
    nodeframe2_pose_.msg.pose.position.y = static_cast<double>(msg->pos_3d[1]);
    nodeframe2_pose_.msg.pose.position.z = static_cast<double>(msg->pos_3d[2]);
    nodeframe2_pose_.publish();
  }

  std::string frame_id_;
  std::map<uint8_t, PosePublisher> anchor_poses_;
  std::map<uint8_t, PosePublisher> nodeframe1_poses_;
  PosePublisher tag_pose_;
  PosePublisher nodeframe2_pose_;

  rclcpp::Subscription<nlink_parser::msg::LinktrackAnchorframe0>::SharedPtr anchor_sub_;
  rclcpp::Subscription<nlink_parser::msg::LinktrackTagframe0>::SharedPtr tag_sub_;
  rclcpp::Subscription<nlink_parser::msg::LinktrackNodeframe1>::SharedPtr nodeframe1_sub_;
  rclcpp::Subscription<nlink_parser::msg::LinktrackNodeframe2>::SharedPtr nodeframe2_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LinktrackRvizConverter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
