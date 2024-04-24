// Copyright 2020 Tier IV, Inc.
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

#include "shift_decider/shift_decider.hpp"

#include <rclcpp/timer.hpp>

#include <cstddef>
#include <functional>
#include <memory>
#include <utility>

ShiftDecider::ShiftDecider(const rclcpp::NodeOptions & node_options)
: Node("shift_decider", node_options)
{
  using std::placeholders::_1;

  static constexpr std::size_t queue_size = 1;
  rclcpp::QoS durable_qos(queue_size);
  durable_qos.transient_local();

  park_on_goal_ = declare_parameter<bool>("park_on_goal");



  pub_shift_cmd_ =
    create_publisher<autoware_auto_vehicle_msgs::msg::GearCommand>("output/gear_cmd", durable_qos);

  // subscriptions
  auto noexec_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  auto noexec_subscription_options = rclcpp::SubscriptionOptions();
  noexec_subscription_options.callback_group = noexec_callback_group; 


  sub_control_cmd_ = create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
    "input/control_cmd", queue_size, std::bind(&ShiftDecider::onControlCmd, this, _1), noexec_subscription_options);
  sub_autoware_state_ = create_subscription<autoware_auto_system_msgs::msg::AutowareState>(
    "input/state", queue_size, std::bind(&ShiftDecider::onAutowareState, this, _1), noexec_subscription_options);
  sub_current_gear_ = create_subscription<autoware_auto_vehicle_msgs::msg::GearReport>(
    "input/current_gear", queue_size, std::bind(&ShiftDecider::onCurrentGear, this, _1), noexec_subscription_options);

  initTimer(0.1);
}

void ShiftDecider::onControlCmd(
  autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg)
{
  assert(false);
  control_cmd_ = msg;
}

void ShiftDecider::onAutowareState(autoware_auto_system_msgs::msg::AutowareState::SharedPtr msg)
{
  assert(false);
  autoware_state_ = msg;
}

void ShiftDecider::onCurrentGear(autoware_auto_vehicle_msgs::msg::GearReport::SharedPtr msg)
{
  assert(false);
  current_gear_ptr_ = msg;
}

void ShiftDecider::takeData() {
  autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr control_cmd_msg = std::make_shared<autoware_auto_control_msgs::msg::AckermannControlCommand>();
  rclcpp::MessageInfo message_info;


  if (sub_control_cmd_->take(*control_cmd_msg, message_info)) {
    control_cmd_ = control_cmd_msg;
  }

  autoware_auto_system_msgs::msg::AutowareState::SharedPtr autoware_state_msg = std::make_shared<autoware_auto_system_msgs::msg::AutowareState>();

  if (sub_autoware_state_->take(*autoware_state_msg, message_info)) {
    autoware_state_ = autoware_state_msg;
  }

  autoware_auto_vehicle_msgs::msg::GearReport::SharedPtr current_gear_msg = std::make_shared<autoware_auto_vehicle_msgs::msg::GearReport>();
  if (sub_current_gear_->take(*current_gear_msg, message_info)) {
    current_gear_ptr_ = current_gear_msg;
  }  

}

void ShiftDecider::onTimer()
{
  takeData();

  if (!autoware_state_ || !control_cmd_ || !current_gear_ptr_) {
    return;
  }

  updateCurrentShiftCmd();
  pub_shift_cmd_->publish(shift_cmd_);
}

void ShiftDecider::updateCurrentShiftCmd()
{
  using autoware_auto_system_msgs::msg::AutowareState;
  using autoware_auto_vehicle_msgs::msg::GearCommand;

  shift_cmd_.stamp = now();
  static constexpr double vel_threshold = 0.01;  // to prevent chattering
  if (autoware_state_->state == AutowareState::DRIVING) {
    if (control_cmd_->longitudinal.speed > vel_threshold) {
      shift_cmd_.command = GearCommand::DRIVE;
    } else if (control_cmd_->longitudinal.speed < -vel_threshold) {
      shift_cmd_.command = GearCommand::REVERSE;
    } else {
      shift_cmd_.command = prev_shift_command;
    }
  } else {
    if (
      (autoware_state_->state == AutowareState::ARRIVED_GOAL ||
       autoware_state_->state == AutowareState::WAITING_FOR_ROUTE) &&
      park_on_goal_) {
      shift_cmd_.command = GearCommand::PARK;
    } else {
      shift_cmd_.command = current_gear_ptr_->report;
    }
  }
  prev_shift_command = shift_cmd_.command;
}

void ShiftDecider::initTimer(double period_s)
{
  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s));
  timer_ =
    rclcpp::create_timer(this, get_clock(), period_ns, std::bind(&ShiftDecider::onTimer, this));
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ShiftDecider)
