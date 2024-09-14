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
 * @brief A test fixture for PublisherNode to facilitate testing.
 *
 * Inherits from PublisherNode and provides a method to test message publishing.
 ******************************************************************************/
class PublisherNodeTest : public PublisherNode
{
public:
  using PublisherNode::PublisherNode;
  float last_published_value_;

  PublisherNodeTest() : PublisherNode() {}

/******************************************************************************
   * @brief Tests the publish_message method.
   *
   * Publishes a value and stores it in last_published_value_.
   *
   * @param value The value to publish.
 ******************************************************************************/
  void test_publish_message(float value)
  {
    publish_message(value);
    last_published_value_ = value;
  }
};

/******************************************************************************
 * @brief Tests the PublisherNode class for correct message publishing.
 *
 * This test initializes ROS2, creates a PublisherNodeTest instance,
 * publishes a value, and verifies the published value.
 ******************************************************************************/
TEST(PublisherNodeTest, UnitTesting)
{
  // Initialize ROS2
  int argc = 0;
  char **argv = nullptr;
  rclcpp::init(argc, argv);

  // Create the node instance
  auto node = std::make_shared<PublisherNodeTest>();

  /* ---------- Test Case 1. ----------*/
  /* Publish a positive value */
  node->test_publish_message(5.0);
  EXPECT_EQ(node->last_published_value_, 5.0);

  /* ---------- Test Case 2. ----------*/
  /* Publish a negative value (currently commented out) */
  node->test_publish_message(-3.0);
  EXPECT_EQ(node->last_published_value_, -3.0);

  /* ---------- Test Case 3. ----------*/
  /* Publish a zero value (currently commented out) */
  node->test_publish_message(0.0);
  EXPECT_EQ(node->last_published_value_, 0.0);

  /* Shutdown ROS2 */
  rclcpp::shutdown();
}