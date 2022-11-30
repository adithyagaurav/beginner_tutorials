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
/**
 * @file File to implement puslisher, service and tf2 broadcaster
 * @author Adithya Singh (agsingh@umd.edu)
 * @brief 
 * @version 0.1
 * @date 2022-11-30
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "beginner_tutorials/srv/custom.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;
using namespace std::placeholders;

/**
 * @brief Publisher class to implement a publisher
 * 
 */

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher(char * transformation[])
  : Node("minimal_publisher"), count_(0) {
    // Create the custom message
    this->message_header = "Custom message ";
    // Declare Frequency parameter
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.description =
        "\nThis parameter modifies the frequency";
    this->declare_parameter("frequency", 10, descriptor);
    // Create broadcaster for tf2
    tf_static_broadcaster_ =
    std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    // Set frequency parameter
    pub_frequency_ = this->get_parameter(
      "frequency").get_parameter_value().get<int>();
    RCLCPP_INFO_STREAM(this->get_logger(), \
    "Setting frequency to: " << pub_frequency_);
    // Bind the callback function to the publisher
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    publisher_ = this->create_publisher<std_msgs::msg::String>(
                                            "chatter", pub_frequency_);
    RCLCPP_DEBUG(this->get_logger(), "Publisher Ready ");
    RCLCPP_WARN(this->get_logger(), \
    "Publisher frequency set to "+std::to_string(pub_frequency_));
    // Create a service server
    service_ = this->create_service<beginner_tutorials::srv::Custom>(
      "CustomString", std::bind(&MinimalPublisher::change_CustomString, \
    this, std::placeholders::_1, std::placeholders::_2));
    // Broadcast the tf2 transform
    this->make_transforms(transformation);
    RCLCPP_DEBUG(this->get_logger(), "Service node ready");
    RCLCPP_DEBUG(this->get_logger(), "NO ERROR");
  }

 private:
 /**
  * @brief Function to broadcast tf2 transform
  * 
  * @param transformation 
  */
  void make_transforms(char * transformation[]) {
    // Create the transofrm message
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = transformation[1];

    t.transform.translation.x = atof(transformation[2]);
    t.transform.translation.y = atof(transformation[3]);
    t.transform.translation.z = atof(transformation[4]);
    tf2::Quaternion q;
    q.setRPY(
      atof(transformation[5]),
      atof(transformation[6]),
      atof(transformation[7]));
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(t);
  }
  /**
   * @brief The function to trigger the ROS2 
   * service to modify string
   * 
   * @param request 
   * @param response 
   */
  void change_CustomString(
    const std::shared_ptr<beginner_tutorials::srv::Custom::Request> request,
    std::shared_ptr<beginner_tutorials::srv::Custom::Response> response) {
      RCLCPP_FATAL(this->get_logger(), \
      "Service call to change message received");
      // Update the string
      response->updated_string = request->input_string;
      this->message_header = response->updated_string;
      RCLCPP_INFO(this->get_logger(), \
      "Modified default message to : ", this->message_header.c_str());
  }

  /**
   * @brief Function to use ROS2 publisher 
   * to publish the message
   * 
   */
  void timer_callback() {
    // Create message
    auto message = std_msgs::msg::String();
    message.data = this->message_header + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    // Publish the message
    publisher_->publish(message);
  }
  /**
   * @brief Timer object to bind publisher
   * 
   */
  rclcpp::TimerBase::SharedPtr timer_;
  /**
   * @brief Publisher to publish messages to the topic
   * 
   */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  /**
   * @brief Service to change custom publishing string
   * 
   */
  rclcpp::Service<beginner_tutorials::srv::Custom>::SharedPtr service_;
  /**
   * @brief Broadcaster to broadcast ROS2 transform
   * 
   */
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  std::string message_header;
  int pub_frequency_;
  size_t count_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>(argv));
  rclcpp::shutdown();
  return 0;
}
