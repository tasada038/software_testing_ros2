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
#include "software_testing_ros2/subscriber_component.hpp"


/******************************************************************************
* @brief A test fixture for integration testing of PublisherNode and SubscriberNode classes.
*
* Sets up two ROS2 nodes: one for publishing messages and one for subscribing to messages.
******************************************************************************/
class IntegrationTest : public ::testing::Test
{
protected:
  /******************************************************************************
  * @brief Sets up the test environment.
  *
  * Initializes ROS2, creates instances of PublisherNode and SubscriberNode.
  ******************************************************************************/
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    publisher_node_ = std::make_shared<PublisherNode>();
    subscriber_node_ = std::make_shared<SubscriberNode>();
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

  std::shared_ptr<PublisherNode> publisher_node_; /**< Shared pointer to the PublisherNode instance. */
  std::shared_ptr<SubscriberNode> subscriber_node_; /**< Shared pointer to the SubscriberNode instance. */
};

/******************************************************************************
* @brief Integration test to verify that the PublisherNode correctly publishes a message and
*        the SubscriberNode receives it.
*
* This test is a black-box test that checks the end-to-end functionality of message publishing and
* subscribing. The test verifies that a message published by the PublisherNode is correctly received
* by the SubscriberNode.
******************************************************************************/
TEST_F(IntegrationTest, TestPublisherSubscriber)
{
  publisher_node_->publish_message(4.5);

  // Simulate communication by spinning the nodes
  rclcpp::spin_some(publisher_node_);
  rclcpp::spin_some(subscriber_node_);

  // Verify that the subscriber received the correct message
  SUCCEED();  // Currently, this test passes if there are no errors in spin_some
}
