/**
 * @file test.cpp
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

#include <catch_ros2/catch_ros2.hpp>
#include <chrono>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;
using std_msgs::msg::String;

auto Logger = rclcpp::get_logger("");

/**
 * @class TestFixture
 * @brief Test Fixture for setting the ROS2 Node and parameters for testing
 */

class TestFixture {
 public:
  /**
   * Constructor to initialize the node and declare the test_duration parameter
   */
  TestFixture() {
    testNode = rclcpp::Node::make_shared("Node1");
    Logger = testNode->get_logger();

    testNode->declare_parameter<double>("test_duration");

    TEST_DURATION = testNode->get_parameter("test_duration")
                        .get_parameter_value()
                        .get<double>();
    RCLCPP_INFO_STREAM(Logger, "Got duration=" << TEST_DURATION);
  }
  /**
   * @brief Destructor for TestFixture Class
   */

  ~TestFixture() {}

 protected:
  // Test duration in seconds
  double TEST_DURATION;
  // Shared pointer to the ROS Node
  rclcpp::Node::SharedPtr testNode;
};

/**
 * @brief Test case to verify message is being published
 *
 * This test subscribes to the "topic" and verifies if a message is received
 * within the specified timeout duration.
 *
 * @param TestFixture The test fixture class.
 * @param "test_talker" The name of the test case.
 * @param "[integration]" The test category used. It can be  [topic] as well
 */
TEST_CASE_METHOD(TestFixture, "test_talker", "[integration]") {
  bool got_topic = false;

  /**
   * @struct ListenerCallback
   * @brief This callback listens for messages and updates the flag
   */
  struct ListenerCallback {
    /**
     * @brief Constructor for the callback
     * @param gotTopic This is a reference to the flag indicating if the message
     * is recieved or not
     */
    explicit ListenerCallback(bool &gotTopic) : gotTopic_(gotTopic) {}
    void operator()(const String msg) const {
      RCLCPP_INFO_STREAM(Logger, "I heard:" << msg.data.c_str());
      gotTopic_ = true;
    }
    bool &gotTopic_;
  };

  // Create a subscriber to the topic with a callback
  auto subscriber = testNode->create_subscription<String>(
      "topic", 10, ListenerCallback(got_topic));

  // Run the test to verify the message is recieved in the time duration
  rclcpp::Rate rate(10.0);  // 10hz checks
  auto start_time = rclcpp::Clock().now();
  auto duration = rclcpp::Clock().now() - start_time;
  auto timeout = rclcpp::Duration::from_seconds(TEST_DURATION);
  RCLCPP_INFO_STREAM(Logger, "duration = " << duration.seconds()
                                           << " timeout=" << timeout.seconds());
  while (!got_topic && (duration < timeout)) {
    rclcpp::spin_some(testNode);
    rate.sleep();
    duration = (rclcpp::Clock().now() - start_time);
  }

  RCLCPP_INFO_STREAM(Logger, "duration = " << duration.seconds()
                                           << " got_topic=" << got_topic);
  // Test assertion to check the topic was received
  CHECK(got_topic);
}
