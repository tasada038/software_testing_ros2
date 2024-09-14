/************************************************************************************/
// MIT License

// Copyright (c) 2024 Takumi Asada

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
/************************************************************************************/

/************************************************************************************/
/*----------------------------------- Include --------------------------------------*/
/************************************************************************************/
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include "software_testing_ros2/publisher_component.hpp"


/******************************************************************************
* @brief A test fixture for integration testing of the PublisherNode class.
*
* Sets up a ROS2 node and a subscription to test message publishing and receiving.
******************************************************************************/
class ComponentTest : public ::testing::Test
{
protected:
  /******************************************************************************
   * @brief Sets up the test environment.
   *
   * Initializes ROS2, creates a PublisherNode instance, and sets up a subscription
   * to listen for Float32 messages on the "float_topic" topic.
  ******************************************************************************/
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<PublisherNode>();
    subscription_ = node_->create_subscription<std_msgs::msg::Float32>(
        "float_topic", 10, [&](const std_msgs::msg::Float32::SharedPtr msg) {
          last_received_value_ = msg->data;
        });
  }

  /******************************************************************************
   * @brief Tears down the test environment.
   *
   * Shuts down ROS2.
  ******************************************************************************/
  void TearDown() override
  {
    rclcpp::shutdown();
  }

  std::shared_ptr<PublisherNode> node_; /**< Shared pointer to the PublisherNode instance. */
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Float32>> subscription_; /**< Subscription to Float32 messages. */
  float last_received_value_; /**< The last received value from the subscription. */
};

/******************************************************************************
* @brief Integration test to verify that the PublisherNode correctly publishes a positive value.
*
* This test is a Black-Box test that checks if a value published by the node is correctly received
* by the subscription. It ensures the end-to-end functionality of the publishing and subscribing system.
******************************************************************************/
TEST_F(ComponentTest, ComponentTestPositive)
{
  node_->publish_message(3.0);
  rclcpp::spin_some(node_);
  EXPECT_EQ(last_received_value_, 3.0);
}

// TEST_F(ComponentTest, ComponentTestNegative)
// {
//   node_->publish_message(-2.0);
//   rclcpp::spin_some(node_);
//   EXPECT_EQ(last_received_value_, -2.0);
// }

// TEST_F(ComponentTest, ComponentTestZero)
// {
//   node_->publish_message(0.0);
//   rclcpp::spin_some(node_);
//   EXPECT_EQ(last_received_value_, 0.0);
// }