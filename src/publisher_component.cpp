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
#include "software_testing_ros2/publisher_component.hpp"


/************************************************************************************/
/*------------------------------------ Class ---------------------------------------*/
/************************************************************************************/
PublisherNode::PublisherNode() : Node("publisher_node")
{
  publisher_ = this->create_publisher<std_msgs::msg::Float32>("float_topic", 10);
}

void PublisherNode::publish_message(float value)
{
  auto msg = std_msgs::msg::Float32();

  if (value > 0.0) {
    RCLCPP_INFO(this->get_logger(), "Publishing positive value: '%f'", value);
    msg.data = value;
  } else if (value < 0.0) {
    RCLCPP_WARN(this->get_logger(), "Publishing negative value: '%f'", value);
    msg.data = value;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Publishing zero value");
    msg.data = 0.0;
  }

  publisher_->publish(msg);
}
