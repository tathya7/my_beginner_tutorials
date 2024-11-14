/**
 * @file subscriber_member_function.cpp
 * @author Tathya Bhatt
 * @version 0.1
 * @copyright Copyright (c) 2024
 *
 */

/**
 * @copyright Copyright 2024 Tathya Bhatt

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
/**
 * @class Subscriber node
 * @brief This class is a ROS2 node with a minimal subscriber, the class
 * inherits from rclcpp::Node class and can subscribve to the topic specified
 */

class MinimalSubscriber : public rclcpp::Node {
 public:
  /**
   * @brief Class constructor
   *
   * This will create the subscriber for a particular topic and captures the
   * information using  callback function
   */
  MinimalSubscriber() : Node("minimal_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

 private:
  /**
   * @brief Callback function which gets invoked whenever the topic is
   * subscribed or recieves new information
   * @param msg indicates the string message published by the publisher
   */
  void topic_callback(const std_msgs::msg::String& msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
/**
 * @brief Main function to initialize the subscriber node
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
