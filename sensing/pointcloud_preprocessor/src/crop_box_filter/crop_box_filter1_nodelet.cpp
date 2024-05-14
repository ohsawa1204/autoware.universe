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
/*
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: cropbox.cpp
 *
 */

#include "pointcloud_preprocessor/crop_box_filter/crop_box_filter1_nodelet.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <vector>

namespace pointcloud_preprocessor
{
CropBoxFilter1Component::CropBoxFilter1Component(const rclcpp::NodeOptions & options)
: Filter1("CropBoxFilter", options)
{
  // initialize debug tool
  {
    using tier4_autoware_utils::DebugPublisher;
    using tier4_autoware_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ = std::make_unique<DebugPublisher>(this, "crop_box_filter");
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }

  // set initial parameters
  {
    auto & p = param_;
    p.min_x = static_cast<float>(declare_parameter("min_x", -1.0));
    p.min_y = static_cast<float>(declare_parameter("min_y", -1.0));
    p.min_z = static_cast<float>(declare_parameter("min_z", -1.0));
    p.max_x = static_cast<float>(declare_parameter("max_x", 1.0));
    p.max_y = static_cast<float>(declare_parameter("max_y", 1.0));
    p.max_z = static_cast<float>(declare_parameter("max_z", 1.0));
    p.negative = static_cast<bool>(declare_parameter("negative", false));
    p.min_x2 = static_cast<float>(declare_parameter("min_x2", -1.0));
    p.min_y2 = static_cast<float>(declare_parameter("min_y2", -1.0));
    p.min_z2 = static_cast<float>(declare_parameter("min_z2", -1.0));
    p.max_x2 = static_cast<float>(declare_parameter("max_x2", 1.0));
    p.max_y2 = static_cast<float>(declare_parameter("max_y2", 1.0));
    p.max_z2 = static_cast<float>(declare_parameter("max_z2", 1.0));
    p.negative2 = static_cast<bool>(declare_parameter("negative2", false));
    p.min_x3 = static_cast<float>(declare_parameter("min_x3", -1.0));
    p.min_y3 = static_cast<float>(declare_parameter("min_y3", -1.0));
    p.min_z3 = static_cast<float>(declare_parameter("min_z3", -1.0));
    p.max_x3 = static_cast<float>(declare_parameter("max_x3", 1.0));
    p.max_y3 = static_cast<float>(declare_parameter("max_y3", 1.0));
    p.max_z3 = static_cast<float>(declare_parameter("max_z3", 1.0));
    p.negative3 = static_cast<bool>(declare_parameter("negative3", false));
    if (tf_input_frame_.empty()) {
      throw std::invalid_argument("Crop box requires non-empty input_frame");
    }
  }

  // set additional publishers
  {
    crop_box_polygon_pub_ =
      this->create_publisher<geometry_msgs::msg::PolygonStamped>("~/crop_box_polygon", 10);
    crop_box_polygon_pub2_ =
      this->create_publisher<geometry_msgs::msg::PolygonStamped>("crop_box_filter_mirror_right/crop_box_polygon", 10);
    crop_box_polygon_pub3_ =
      this->create_publisher<geometry_msgs::msg::PolygonStamped>("crop_box_filter_mirror_left/crop_box_polygon", 10);
  }

  // set parameter service callback
  {
    using std::placeholders::_1;
    set_param_res_ = this->add_on_set_parameters_callback(
      std::bind(&CropBoxFilter1Component::paramCallback, this, _1));
  }
}

// TODO(sykwer): Temporary Implementation: Delete this function definition when all the filter nodes
// conform to new API.
void CropBoxFilter1Component::filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  (void)input;
  (void)indices;
  (void)output;
}

void CropBoxFilter1Component::faster_filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output,
  const TransformInfo & transform_info)
{
    auto mid1 = std::make_unique<PointCloud2>();
    auto mid2 = std::make_unique<PointCloud2>();
    auto mid3 = std::make_unique<PointCloud2>();
    TransformInfo __transform_info;

    do_faster_filter(input, indices, *mid1, transform_info, 1);
    if (!convert_output_costly(mid1))
        return;
    mid1->header.stamp = input->header.stamp;

    if (!isValid(reinterpret_cast<const PointCloud2ConstPtr &>(mid1)))
        return;
    if (indices && !isValid(reinterpret_cast<const PointIndicesConstPtr &>(indices)))
        return;
    tf_input_orig_frame_ = mid1->header.frame_id;
    if (!calculate_transform_matrix(tf_input_frame_, *mid1, __transform_info))
        return;

    do_faster_filter(reinterpret_cast<const PointCloud2ConstPtr &>(mid1), indices, *mid2, __transform_info, 2);
    if (!convert_output_costly(mid2))
        return;
    mid2->header.stamp = mid1->header.stamp;

    if (!isValid(reinterpret_cast<const PointCloud2ConstPtr &>(mid2)))
        return;
    if (indices && !isValid(reinterpret_cast<const PointIndicesConstPtr &>(indices)))
        return;
    tf_input_orig_frame_ = mid2->header.frame_id;
    if (!calculate_transform_matrix(tf_input_frame_, *mid2, __transform_info))
        return;

    do_faster_filter(reinterpret_cast<const PointCloud2ConstPtr &>(mid2), indices, *mid3, __transform_info, 3);
    if (!convert_output_costly(mid3))
        return;
    mid3->header.stamp = mid2->header.stamp;

    pub_output_->publish(std::move(mid3));
}

// TODO(sykwer): Temporary Implementation: Rename this function to `filter()` when all the filter
// nodes conform to new API. Then delete the old `filter()` defined above.
void CropBoxFilter1Component::do_faster_filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output,
  const TransformInfo & transform_info, int index)
{
  std::scoped_lock lock(mutex_);
  stop_watch_ptr_->toc("processing_time", true);

  if (indices) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "Indices are not supported and will be ignored");
  }

  int x_offset = input->fields[pcl::getFieldIndex(*input, "x")].offset;
  int y_offset = input->fields[pcl::getFieldIndex(*input, "y")].offset;
  int z_offset = input->fields[pcl::getFieldIndex(*input, "z")].offset;

  output.data.resize(input->data.size());
  size_t output_size = 0;

  int skipped_count = 0;

  for (size_t global_offset = 0; global_offset + input->point_step <= input->data.size();
       global_offset += input->point_step) {
    Eigen::Vector4f point;
    std::memcpy(&point[0], &input->data[global_offset + x_offset], sizeof(float));
    std::memcpy(&point[1], &input->data[global_offset + y_offset], sizeof(float));
    std::memcpy(&point[2], &input->data[global_offset + z_offset], sizeof(float));
    point[3] = 1;

    if (!std::isfinite(point[0]) || !std::isfinite(point[1]) || !std::isfinite(point[2])) {
      skipped_count++;
      continue;
    }

    if (transform_info.need_transform) {
      point = transform_info.eigen_transform * point;
    }

    if (index == 1) {
    bool point_is_inside = point[2] > param_.min_z && point[2] < param_.max_z &&
                           point[1] > param_.min_y && point[1] < param_.max_y &&
                           point[0] > param_.min_x && point[0] < param_.max_x;
    if ((!param_.negative && point_is_inside) || (param_.negative && !point_is_inside)) {
      memcpy(&output.data[output_size], &input->data[global_offset], input->point_step);

      if (transform_info.need_transform) {
        std::memcpy(&output.data[output_size + x_offset], &point[0], sizeof(float));
        std::memcpy(&output.data[output_size + y_offset], &point[1], sizeof(float));
        std::memcpy(&output.data[output_size + z_offset], &point[2], sizeof(float));
      }

      output_size += input->point_step;
    }
    } else if (index == 2) {
    bool point_is_inside = point[2] > param_.min_z2 && point[2] < param_.max_z2 &&
                           point[1] > param_.min_y2 && point[1] < param_.max_y2 &&
                           point[0] > param_.min_x2 && point[0] < param_.max_x2;
    if ((!param_.negative2 && point_is_inside) || (param_.negative2 && !point_is_inside)) {
      memcpy(&output.data[output_size], &input->data[global_offset], input->point_step);

      if (transform_info.need_transform) {
        std::memcpy(&output.data[output_size + x_offset], &point[0], sizeof(float));
        std::memcpy(&output.data[output_size + y_offset], &point[1], sizeof(float));
        std::memcpy(&output.data[output_size + z_offset], &point[2], sizeof(float));
      }

      output_size += input->point_step;
    }
    } else if (index == 3) {
    bool point_is_inside = point[2] > param_.min_z3 && point[2] < param_.max_z3 &&
                           point[1] > param_.min_y3 && point[1] < param_.max_y3 &&
                           point[0] > param_.min_x3 && point[0] < param_.max_x3;
    if ((!param_.negative3 && point_is_inside) || (param_.negative3 && !point_is_inside)) {
      memcpy(&output.data[output_size], &input->data[global_offset], input->point_step);

      if (transform_info.need_transform) {
        std::memcpy(&output.data[output_size + x_offset], &point[0], sizeof(float));
        std::memcpy(&output.data[output_size + y_offset], &point[1], sizeof(float));
        std::memcpy(&output.data[output_size + z_offset], &point[2], sizeof(float));
      }

      output_size += input->point_step;
    }
    }

  }

  if (skipped_count > 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "%d points contained NaN values and have been ignored",
      skipped_count);
  }

  output.data.resize(output_size);

  // Note that tf_input_orig_frame_ is the input frame, while tf_input_frame_ is the frame of the
  // crop box
  output.header.frame_id = tf_input_frame_;

  output.height = 1;
  output.fields = input->fields;
  output.is_bigendian = input->is_bigendian;
  output.point_step = input->point_step;
  output.is_dense = input->is_dense;
  output.width = static_cast<uint32_t>(output.data.size() / output.height / output.point_step);
  output.row_step = static_cast<uint32_t>(output.data.size() / output.height);

  publishCropBoxPolygon(index);

  // add processing time for debug
  if (debug_publisher_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
  }
}

void CropBoxFilter1Component::publishCropBoxPolygon(int index)
{
  auto generatePoint = [](double x, double y, double z) {
    geometry_msgs::msg::Point32 point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
  };

  double x1 = param_.max_x;
  double x2 = param_.min_x;
  double x3 = param_.min_x;
  double x4 = param_.max_x;

  double y1 = param_.max_y;
  double y2 = param_.max_y;
  double y3 = param_.min_y;
  double y4 = param_.min_y;

  double z1 = param_.min_z;
  double z2 = param_.max_z;

  if (index == 2) {
      x1 = param_.max_x2;
      x2 = param_.min_x2;
      x3 = param_.min_x2;
      x4 = param_.max_x2;

      y1 = param_.max_y2;
      y2 = param_.max_y2;
      y3 = param_.min_y2;
      y4 = param_.min_y2;

      z1 = param_.min_z2;
      z2 = param_.max_z2;
  }
  else if (index == 3) {
      x1 = param_.max_x3;
      x2 = param_.min_x3;
      x3 = param_.min_x3;
      x4 = param_.max_x3;

      y1 = param_.max_y3;
      y2 = param_.max_y3;
      y3 = param_.min_y3;
      y4 = param_.min_y3;

      z1 = param_.min_z3;
      z2 = param_.max_z3;
  }

  geometry_msgs::msg::PolygonStamped polygon_msg;
  polygon_msg.header.frame_id = tf_input_frame_;
  polygon_msg.header.stamp = get_clock()->now();
  polygon_msg.polygon.points.push_back(generatePoint(x1, y1, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x2, y2, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x3, y3, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x4, y4, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x1, y1, z1));

  polygon_msg.polygon.points.push_back(generatePoint(x1, y1, z2));

  polygon_msg.polygon.points.push_back(generatePoint(x2, y2, z2));
  polygon_msg.polygon.points.push_back(generatePoint(x2, y2, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x2, y2, z2));

  polygon_msg.polygon.points.push_back(generatePoint(x3, y3, z2));
  polygon_msg.polygon.points.push_back(generatePoint(x3, y3, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x3, y3, z2));

  polygon_msg.polygon.points.push_back(generatePoint(x4, y4, z2));
  polygon_msg.polygon.points.push_back(generatePoint(x4, y4, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x4, y4, z2));

  polygon_msg.polygon.points.push_back(generatePoint(x1, y1, z2));

  if (index == 1)
      crop_box_polygon_pub_->publish(polygon_msg);
  else if (index == 2)
      crop_box_polygon_pub2_->publish(polygon_msg);
  else if (index == 3)
      crop_box_polygon_pub3_->publish(polygon_msg);
}

rcl_interfaces::msg::SetParametersResult CropBoxFilter1Component::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mutex_);

  CropBoxParam new_param{};

  if (
    get_param(p, "min_x", new_param.min_x) && get_param(p, "min_y", new_param.min_y) &&
    get_param(p, "min_z", new_param.min_z) && get_param(p, "max_x", new_param.max_x) &&
    get_param(p, "max_y", new_param.max_y) && get_param(p, "max_z", new_param.max_z) &&
    get_param(p, "negative", new_param.negative)) {
    if (
      param_.min_x != new_param.min_x || param_.max_x != new_param.max_x ||
      param_.min_y != new_param.min_y || param_.max_y != new_param.max_y ||
      param_.min_z != new_param.min_z || param_.max_z != new_param.max_z ||
      param_.negative != new_param.negative) {
      RCLCPP_DEBUG(
        get_logger(), "[%s::paramCallback] Setting the minimum point to: %f %f %f.", get_name(),
        new_param.min_x, new_param.min_y, new_param.min_z);
      RCLCPP_DEBUG(
        get_logger(), "[%s::paramCallback] Setting the minimum point to: %f %f %f.", get_name(),
        new_param.max_x, new_param.max_y, new_param.max_z);
      RCLCPP_DEBUG(
        get_logger(), "[%s::paramCallback] Setting the filter negative flag to: %s.", get_name(),
        new_param.negative ? "true" : "false");
      param_ = new_param;
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

}  // namespace pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::CropBoxFilter1Component)
