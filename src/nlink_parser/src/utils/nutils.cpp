#include "nutils.h"

#include <rclcpp/logging.hpp>

void TopicAdvertisedTip(const rclcpp::Logger &logger, const char *topic) {
  RCLCPP_INFO(logger,
              "%s has been advertised, use 'ros2 topic echo %s' to view the data",
              topic, topic);
}
