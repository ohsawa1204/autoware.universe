// Copyright 2023 TIER IV, Inc.
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

#include "yabloc_common/static_tf_subscriber.hpp"

std::mutex ___global_mutex __attribute__((weak));
std::shared_ptr<tf2_ros::Buffer> ___global_tf_buffer_ __attribute__((weak));
std::shared_ptr<tf2_ros::TransformListener> ___global_tf_listener_ __attribute__((weak));

namespace yabloc::common
{
StaticTfSubscriber::StaticTfSubscriber(rclcpp::Clock::SharedPtr clock)
{
  std::lock_guard<std::mutex> lock(___global_mutex);
  if (___global_tf_buffer_ == nullptr) {
    ___global_tf_buffer_ = tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock);
    ___global_tf_listener_ = transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  } else {
    tf_buffer_ = ___global_tf_buffer_;
    transform_listener_ = ___global_tf_listener_;
  }
}

std::optional<Sophus::SE3f> StaticTfSubscriber::se3f(
  const std::string & frame_id, const std::string & parent_frame_id)
{
  std::optional<Eigen::Affine3f> opt_aff = (*this)(frame_id, parent_frame_id);
  if (!opt_aff.has_value()) return std::nullopt;

  Sophus::SE3f se3f(opt_aff->rotation(), opt_aff->translation());
  return se3f;
}

std::optional<Eigen::Affine3f> StaticTfSubscriber::operator()(
  const std::string & frame_id, const std::string & parent_frame_id)
{
  std::optional<Eigen::Affine3f> extrinsic_{std::nullopt};
  try {
    geometry_msgs::msg::TransformStamped ts =
      tf_buffer_->lookupTransform(parent_frame_id, frame_id, tf2::TimePointZero);
    Eigen::Vector3f p;
    p.x() = ts.transform.translation.x;
    p.y() = ts.transform.translation.y;
    p.z() = ts.transform.translation.z;

    Eigen::Quaternionf q;
    q.w() = ts.transform.rotation.w;
    q.x() = ts.transform.rotation.x;
    q.y() = ts.transform.rotation.y;
    q.z() = ts.transform.rotation.z;
    extrinsic_ = Eigen::Affine3f::Identity();
    extrinsic_->translation() = p;
    extrinsic_->matrix().topLeftCorner(3, 3) = q.toRotationMatrix();
  } catch (tf2::TransformException & ex) {
  }
  return extrinsic_;
}

}  // namespace yabloc::common
