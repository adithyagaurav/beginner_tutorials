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
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "beginner_tutorials/srv/custom.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0) {
    this->message_header = "Custom message ";
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description =
        "\nThis parameter modifies the frequency";
    this->declare_parameter("frequency", 10, descriptor);
    pub_frequency_ = this->get_parameter(
      "frequency").get_parameter_value().get<int>();
    RCLCPP_INFO_STREAM(this->get_logger(), \
    "Setting frequency to: " << pub_frequency_);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    publisher_ = this->create_publisher<std_msgs::msg::String>(
                                            "topic", pub_frequency_);
    RCLCPP_DEBUG(this->get_logger(), "Publisher Ready ");
    RCLCPP_WARN(this->get_logger(), \
    "Publisher frequency set to "+std::to_string(pub_frequency_));
    service_ = this->create_service<beginner_tutorials::srv::Custom>(
      "CustomString", std::bind(&MinimalPublisher::change_CustomString, \
    this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_DEBUG(this->get_logger(), "Service node ready");
    RCLCPP_DEBUG(this->get_logger(), "NO ERROR");
  }

 private:
  void change_CustomString(
    const std::shared_ptr<beginner_tutorials::srv::Custom::Request> request,
    std::shared_ptr<beginner_tutorials::srv::Custom::Response> response) {
      RCLCPP_FATAL(this->get_logger(), \
      "Service call to change message received");
      response->updated_string = request->input_string;
      this->message_header = response->updated_string;
      RCLCPP_INFO(this->get_logger(), \
      "Modified default message to : ", this->message_header.c_str());
    }

  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = this->message_header + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<beginner_tutorials::srv::Custom>::SharedPtr service_;
  std::string message_header;
  int pub_frequency_;
  size_t count_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
