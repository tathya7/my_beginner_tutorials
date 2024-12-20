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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <beginner_tutorials/srv/change_string.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <string>

#include "beginner_tutorials/srv/detail/change_string__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/**
 * @class TextPublisher
 * @brief ROS2 Node which publishes text message and a service is added which
 * can change the message. The node also includes the broadcaster
 */

class TextPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for the class object which initializes the nnode and
   * creates a publisher, service and timer callback
   */
  TextPublisher() : Node("text_publisher"), count_(0) {
    // declare the parameter publish frequency parameter with default value on
    // current instance
    this->declare_parameter("publish_frequency", 300);
    // parameter to record a bag
    this->declare_parameter("record_bag", 1);

    // Opening the rosbag writer
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open("my_bag");

    // Publisher and service initializing
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    service_ = this->create_service<beginner_tutorials::srv::ChangeString>(
        "change_string",
        std::bind(&TextPublisher::change_str, this, std::placeholders::_1,
                  std::placeholders::_2));

    if (this->get_parameter("publish_frequency").as_int() == 300) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Changing publish frequency");
    }

    // Creating the static broadcaster
    tf_static_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Setting the timer which triggers the callback function
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(
            this->get_parameter("publish_frequency").as_int()),
        std::bind(&TextPublisher::timer_callback, this));
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
    this->broadcast_static_transform();

    if (this->get_parameter("record_bag").as_int() == 1) {
      rclcpp::Time time_stamp = this->now();
      writer_->write(message, "topic", time_stamp);
    }
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
  /**
   * @brief Member function to broadcast a static transform by translating and
   * rotating and converts the RPY to quaternion. The function then broadcasts
   * the message to the "/tf_static" topic
   */

  void broadcast_static_transform() {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "talk";

    t.transform.translation.x = 1.0;
    t.transform.translation.y = 4.0;
    t.transform.translation.z = 2.0;
    tf2::Quaternion q;
    q.setRPY(2.0, 6.0, 8.0);

    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(t);

    if (this->get_parameter("record_bag").as_int() == 1) {
      rclcpp::Time time_stamp = this->now();
      writer_->write(t, "topic", time_stamp);
    }
  }

  std_msgs::msg::String message;

  // Creating a service_ attribute
  rclcpp::Service<beginner_tutorials::srv::ChangeString>::SharedPtr service_;

  // Timer attribute for publisher
  rclcpp::TimerBase::SharedPtr timer_;

  // Creating publisher attrribute
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  // Used for keeping count of the timer
  size_t count_;

  // Creating broadcaster of the shared pointer to StaticTransformBroadccaster
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

  // Unique pointer to write and save the bag
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
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
