/**
 * @file publisher_member_function.cpp
 * @author Tathya Bhatt
 * @version 0.1
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

#include <beginner_tutorials/srv/change_string.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "beginner_tutorials/srv/detail/change_string__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/**
 * @class TextPublisher
 * @brief ROS2 Node which publishes text message and a service is added which
 * can change the message
 */

class TextPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for the class object which initializes the nnode and
   * creates a publisher, service and timer callback
   */
  TextPublisher() : Node("text_publisher"), count_(0) {
    this->declare_parameter("publish_frequency", 300);
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    service_ = this->create_service<beginner_tutorials::srv::ChangeString>(
        "change_string",
        std::bind(&TextPublisher::change_str, this, std::placeholders::_1,
                  std::placeholders::_2));

    if (this->get_parameter("publish_frequency").as_int() == 300) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Changing publish frequency");
    }

    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);


    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(
            this->get_parameter("publish_frequency").as_int()),
        std::bind(&TextPublisher::timer_callback, this));

    this->broadcast_static_transform();
  }

 private:
  /**
   * @brief Callback functions for timer which publishes message at every
   * milliseconds
   */
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "This is assignment 1 " + std::to_string(count_++);
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: " << message.data);
    publisher_->publish(message);
  }
  /**
   * @brief Service callback function
   * @param request The request which the service processes which is to change
   * the string
   * @param response The service response after changing the string
   */

  void change_str(
      const std::shared_ptr<beginner_tutorials::srv::ChangeString::Request>
          request,
      std::shared_ptr<beginner_tutorials::srv::ChangeString::Response> resp) {
    this->message.data = request->new_string;
    resp->string_change_status = request->new_string;
    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "Received Service Request: " << request->new_string);
  }

  void broadcast_static_transform(){
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "talk";

    t.transform.translation.x = 1.0;
    t.transform.translation.y = 4.0;
    t.transform.translation.z = 2.0;
    tf2::Quaternion q;
    q.setRPY(2.0,6.0,8.0);

    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(t);

  }

  std_msgs::msg::String message;
  rclcpp::Service<beginner_tutorials::srv::ChangeString>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

};

/**
 * @brief Main function call
 * @param argc,argv Command line argument

 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TextPublisher>();
  RCLCPP_FATAL_STREAM(node->get_logger(), "This is a example fatal message");
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
