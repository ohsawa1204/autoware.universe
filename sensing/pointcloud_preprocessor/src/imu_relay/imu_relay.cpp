// Copyright 2024 TIER IV, Inc.
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

#include "pointcloud_preprocessor/imu_relay/imu_relay.hpp"

namespace pointcloud_preprocessor
{
ImuRelayComponent::ImuRelayComponent(const rclcpp::NodeOptions & node_options)
: Node("imu_relay_node", node_options)
{
  using std::placeholders::_1;

  batch_imu_msg.imu.resize(20);
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "~/input/imu", 10,
    std::bind(&ImuRelayComponent::onImu, this, std::placeholders::_1));
  batch_imu_pub_ = this->create_publisher<autoware_sensing_msgs::msg::BatchImu>("~/output/batch_imu", 10);
}

void ImuRelayComponent::onImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
{
  static int num_accumulated;

  batch_imu_msg.imu[num_accumulated] = *imu_msg;
  num_accumulated++;
  if (num_accumulated == 20) {
    num_accumulated = 0;
    batch_imu_pub_->publish(batch_imu_msg);
  }
}
}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::ImuRelayComponent)
