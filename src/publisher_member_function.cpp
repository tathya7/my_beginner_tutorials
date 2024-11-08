// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>
#include <string>

#include "beginner_tutorials/srv/detail/change_string__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <beginner_tutorials/srv/change_string.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class TextPublisher : public rclcpp::Node {
 public:
  TextPublisher() : Node("text_publisher"), count_(0) {
    this->declare_parameter("publish_frequency", 600);
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    service_ = this->create_service<beginner_tutorials::srv::ChangeString>(
        "change_string",
        std::bind(&TextPublisher::change_str, this,
                  std::placeholders::_1, std::placeholders::_2));


    if(this->get_parameter("publish_frequency").as_int()==500){
      RCLCPP_WARN_STREAM(this->get_logger(), "Changing publish frequency");
    };

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(this->get_parameter("publish_frequency").as_int()), std::bind(&TextPublisher::timer_callback, this));
  }

 private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "This is assignment 1 " + std::to_string(count_++);
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: "<< message.data);
    publisher_->publish(message);
  }

  void change_str(
      const std::shared_ptr<beginner_tutorials::srv::ChangeString::Request>
          request,
      std::shared_ptr<beginner_tutorials::srv::ChangeString::Response> resp) {
    this->message.data = request->new_string;
    resp->string_change_status = request->new_string;
    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "Received Service Request: " << request->new_string);
  }

  std_msgs::msg::String message;
  rclcpp::Service<beginner_tutorials::srv::ChangeString>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TextPublisher>();
  rclcpp::spin(node);

  RCLCPP_FATAL_STREAM(node->get_logger(), "This is a example fatal message");
  rclcpp::shutdown();
  return 0;
}
