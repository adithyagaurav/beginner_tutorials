// Copyright 2015 Open Source Robotics Foundation, Inc.
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
#include <memory>

#include "gtest/gtest.h"

#include "rclcpp/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"

#include "beginner_tutorials/srv/custom.hpp"

#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

using namespace std::chrono_literals;

class CLASSNAME (test_services_client, RMW_IMPLEMENTATION) : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }
};

TEST_F(CLASSNAME(test_services_client, RMW_IMPLEMENTATION), test_add_noreqid) {
  auto node = rclcpp::Node::make_shared("test_services_client_no_reqid");

  auto client = node->create_client<beginner_tutorials::srv::Custom>("CustomString");
  auto request = std::make_shared<beginner_tutorials::srv::Custom::Request>();
  std::string new_string = "New string";
  request->input_string = new_string;

  if (!client->wait_for_service(500s)) {
    ASSERT_TRUE(false) << "service not available after waiting";
  }
  std::cout<<"*******"<<std::endl;
  auto result = client->async_send_request(request);

  auto ret = rclcpp::spin_until_future_complete(node, result, 5s);  // Wait for the result.
  ASSERT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);

  EXPECT_EQ(request->input_string.c_str(), result.get()->updated_string.c_str());
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  std::cout << "DONE SHUTTING DOWN ROS" << std::endl;
  return result;
}