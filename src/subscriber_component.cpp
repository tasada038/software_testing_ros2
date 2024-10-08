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
#include "software_testing_ros2/subscriber_component.hpp"

/************************************************************************************/
/*------------------------------------ Class ---------------------------------------*/
/************************************************************************************/
SubscriberNode::SubscriberNode() : Node("subscriber_node")
{
  subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "float_topic", 10, std::bind(&SubscriberNode::callback, this, std::placeholders::_1));
}

void SubscriberNode::callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received: '%f'", msg->data);
}
