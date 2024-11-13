#include <catch_ros2/catch_ros2.hpp>
#include <chrono>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs//msg/string.hpp>

using namespace std::chrono_literals;
using std_msgs::msg::String;

auto Logger = rclcpp::get_logger ("");

class TestFixture {
    public:
    TestFixture()
    {
        testNode = rclcpp::Node::make_shared("Node1");
        Logger = testNode->get_logger();

        testNode->declare_parameter<double>("test_duration");

        TEST_DURATION = testNode->get_parameter("test_duration").get_parameter_value().get<double>();
        RCLCPP_INFO_STREAM (Logger, "Got duration=" << TEST_DURATION);
    }

    ~TestFixture(){}

    protected:
    double TEST_DURATION;
    rclcpp::Node::SharedPtr testNode;
};

TEST_CASE_METHOD (TestFixture, "test_talker", "[integration]"){

    /**
   * 4.) Now, subscribe to a specific topic we're looking for:
   */
  bool got_topic = false;

  // Define a callback that captures the additional parameter
  struct ListenerCallback {
    ListenerCallback(bool &gotTopic) : gotTopic_(gotTopic)
    {}
    void operator()(const String msg) const {
      RCLCPP_INFO_STREAM (Logger, "I heard:" << msg.data.c_str());
      gotTopic_ = true;
    }
    bool &gotTopic_;
  };

  auto subscriber = testNode->create_subscription<String> ("topic", 10, ListenerCallback (got_topic));

  /**
   * 5.) Finally do the actual test:
   */
  rclcpp::Rate rate(10.0);       // 10hz checks
  auto start_time = rclcpp::Clock().now();
  auto duration   = rclcpp::Clock().now() - start_time;
  auto timeout    = rclcpp::Duration::from_seconds(TEST_DURATION);
  RCLCPP_INFO_STREAM (Logger, "duration = " << duration.seconds() << " timeout=" << timeout.seconds());
  while (!got_topic && (duration < timeout))
    {
      rclcpp::spin_some (testNode);
      rate.sleep();
      duration = (rclcpp::Clock().now() - start_time);
    }
  
  RCLCPP_INFO_STREAM (Logger, "duration = " << duration.seconds() << " got_topic=" << got_topic);
  CHECK (got_topic); // Test assertions - check that the topic was received
 }

