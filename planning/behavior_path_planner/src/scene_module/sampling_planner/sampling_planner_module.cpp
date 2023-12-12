// Copyright 2021 Tier IV, Inc.
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

#include "behavior_path_planner/scene_module/sampling_planner/sampling_planner_module.hpp"

#include <boost/geometry/algorithms/within.hpp>

namespace behavior_path_planner
{
using geometry_msgs::msg::Point;
using motion_utils::calcSignedArcLength;
using motion_utils::findNearestIndex;
using motion_utils::findNearestSegmentIndex;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::calcOffsetPose;
using tier4_autoware_utils::getPoint;
using tier4_autoware_utils::Point2d;
using utils::toPath;

SamplingPlannerModule::SamplingPlannerModule(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<SamplingPlannerParameters> & parameters,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map)
: SceneModuleInterface{name, node, rtc_interface_ptr_map},
  vehicle_info_{vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo()}
{
  internal_params_ =
    std::shared_ptr<SamplingPlannerInternalParameters>(new SamplingPlannerInternalParameters{});
  updateModuleParams(parameters);

  hard_constraints_.emplace_back(
    [](
      sampler_common::Path & path, const sampler_common::Constraints & constraints,
      const MultiPoint2d & footprint) -> bool {
      if (!footprint.empty()) {
        path.constraint_results.drivable_area =
          boost::geometry::within(footprint, constraints.drivable_polygons);
      }

      path.constraint_results.collision =
        !sampler_common::constraints::has_collision(footprint, constraints.obstacle_polygons);
      return path.constraint_results.collision && path.constraint_results.drivable_area;
    });

  hard_constraints_.emplace_back(
    [](
      sampler_common::Path & path, const sampler_common::Constraints & constraints,
      [[maybe_unused]] const MultiPoint2d & footprint) -> bool {
      if (path.curvatures.empty()) return true;
      const bool curvatures_satisfied =
        std::all_of(path.curvatures.begin(), path.curvatures.end(), [&](const auto & v) -> bool {
          return (v > constraints.hard.min_curvature) && (v < constraints.hard.max_curvature);
        });
      path.constraint_results.curvature = curvatures_satisfied;
      return curvatures_satisfied;
    });

  // TODO(Daniel): Maybe add a soft cost for average distance to centerline?
  // TODO(Daniel): Think of methods to prevent chattering

  //  Yaw difference
  // soft_constraints_.emplace_back(
  //   [&](
  //     sampler_common::Path & path, [[maybe_unused]] const sampler_common::Constraints &
  //     constraints,
  //     [[maybe_unused]] const SoftConstraintsInputs & input_data) -> double {
  //     if (path.points.empty()) return 0.0;
  //     const auto & goal_pose_yaw =
  //     tier4_autoware_utils::getRPY(input_data.goal_pose.orientation).z; const auto &
  //     last_point_yaw = path.yaws.back(); const double angle_difference = std::abs(last_point_yaw
  //     - goal_pose_yaw); return angle_difference / (3.141519 / 4.0);
  //   });

  //  Distance to goal
  soft_constraints_.emplace_back(
    [&](
      sampler_common::Path & path, [[maybe_unused]] const sampler_common::Constraints & constraints,
      [[maybe_unused]] const SoftConstraintsInputs & input_data) -> double {
      if (path.points.empty()) return 0.0;
      if (!std::any_of(
            input_data.current_lanes.begin(), input_data.current_lanes.end(),
            [&](const auto & lane) {
              return lanelet::utils::isInLanelet(input_data.goal_pose, lane);
            }))
        return 0.0;
      if (path.poses.empty()) return 0.0;
      const auto & goal_pose = input_data.goal_pose;
      const auto & ego_pose = input_data.ego_pose;
      const auto & path_last_pose = path.poses.back();

      const auto ego_arc = lanelet::utils::getArcCoordinates(input_data.current_lanes, ego_pose);
      const auto goal_arc = lanelet::utils::getArcCoordinates(input_data.current_lanes, goal_pose);
      const auto path_last_arc =
        lanelet::utils::getArcCoordinates(input_data.current_lanes, path_last_pose);

      const double distance_ego_to_goal_pose = std::abs(ego_arc.length - goal_arc.length);

      constexpr double epsilon = 0.001;
      if (distance_ego_to_goal_pose < epsilon) return 0.0;
      const double length_path_to_goal = std::abs(path_last_arc.length - goal_arc.length);
      const double lateral_distance_to_goal = std::abs(path_last_arc.distance - goal_arc.distance);

      const double max_target_distance = *std::max_element(
        internal_params_->sampling.target_lateral_positions.begin(),
        internal_params_->sampling.target_lateral_positions.end());

      const double max_target_length = *std::max_element(
        internal_params_->sampling.target_lengths.begin(),
        internal_params_->sampling.target_lengths.end());

      const double reference_max_distance = std::hypot(max_target_length, max_target_distance);

      return (length_path_to_goal / reference_max_distance) +
             (lateral_distance_to_goal /
              reference_max_distance);  // + (angle_difference / (pi / 4.0));
    });

  // Distance to centerline
  soft_constraints_.emplace_back(
    [&](
      [[maybe_unused]] sampler_common::Path & path,
      [[maybe_unused]] const sampler_common::Constraints & constraints,
      [[maybe_unused]] const SoftConstraintsInputs & input_data) -> double {
      if (path.poses.empty()) return 0.0;
      if (!std::any_of(
            input_data.current_lanes.begin(), input_data.current_lanes.end(),
            [&](const auto & lane) {
              return lanelet::utils::isInLanelet(input_data.goal_pose, lane);
            }))
        return 0.0;

      const auto & goal_pose = input_data.goal_pose;
      const auto goal_arc = lanelet::utils::getArcCoordinates(input_data.current_lanes, goal_pose);
      const double min_target_length = *std::min_element(
        internal_params_->sampling.target_lengths.begin(),
        internal_params_->sampling.target_lengths.end());

      // double distance_average{0.0};
      // for (const auto pose : path.poses) {
      //   const auto & path_point_arc =
      //     lanelet::utils::getArcCoordinates(input_data.current_lanes, pose);
      //   const double distance_point_to_goal = std::abs(path_point_arc.length - goal_arc.length);
      //   const double lateral_distance_to_center_lane =
      //     (distance_point_to_goal < min_target_length) ? 0.0 : std::abs(path_point_arc.distance);
      //   distance_average += lateral_distance_to_center_lane;
      // }
      const auto & path_point_arc =
        lanelet::utils::getArcCoordinates(input_data.current_lanes, path.poses.back());
      const double distance_point_to_goal = std::abs(path_point_arc.length - goal_arc.length);
      const double lateral_distance_to_center_lane =
        (distance_point_to_goal < min_target_length) ? 0.0 : std::abs(path_point_arc.distance);
      // distance_average += lateral_distance_to_center_lane;
      // distance_average = distance_average / path.poses.size();
      const double acceptable_width = constraints.ego_width / 2.0;
      // return distance_average / acceptable_width;
      return lateral_distance_to_center_lane / acceptable_width;
    });

  // // Curvature cost
  soft_constraints_.emplace_back(
    [](
      sampler_common::Path & path, [[maybe_unused]] const sampler_common::Constraints & constraints,
      [[maybe_unused]] const SoftConstraintsInputs & input_data) -> double {
      if (path.curvatures.empty()) return std::numeric_limits<double>::max();
      double curvature_sum = 0.0;
      for (const auto curvature : path.curvatures) {
        curvature_sum += std::abs(curvature);
      }
      const auto curvature_average = curvature_sum / static_cast<double>(path.curvatures.size());
      const auto max_curvature =
        (curvature_average > 0.0) ? constraints.hard.max_curvature : constraints.hard.min_curvature;
      return 1000.0 * curvature_average / max_curvature;
      // return constraints.soft.curvature_weight * curvature_sum /
      //        static_cast<double>(path.curvatures.size());
    });
}

bool SamplingPlannerModule::isExecutionRequested() const
{
  if (getPreviousModuleOutput().reference_path->points.empty()) {
    return false;
  }

  if (!motion_utils::isDrivingForward(getPreviousModuleOutput().reference_path->points)) {
    RCLCPP_WARN(getLogger(), "Backward path is NOT supported. Just converting path to trajectory");
    return false;
  }

  return !isReferencePathSafe();
}

bool SamplingPlannerModule::isReferencePathSafe() const
{
  std::vector<DrivableLanes> drivable_lanes{};

  const auto & prev_module_path = getPreviousModuleOutput().path;
  const auto & current_lanes = utils::getCurrentLanesFromPath(*prev_module_path, planner_data_);
  // expand drivable lanes
  std::for_each(current_lanes.begin(), current_lanes.end(), [&](const auto & lanelet) {
    auto d = generateExpandDrivableLanes(lanelet, planner_data_);
    drivable_lanes.push_back(d);
  });

  {
    const auto left_bound =
      (utils::calcBound(planner_data_->route_handler, drivable_lanes, false, false, true));
    const auto right_bound =
      (utils::calcBound(planner_data_->route_handler, drivable_lanes, false, false, false));

    const auto sampling_planner_data =
      createPlannerData(planner_data_->prev_output_path, left_bound, right_bound);

    prepareConstraints(
      internal_params_->constraints, planner_data_->dynamic_object,
      sampling_planner_data.left_bound, sampling_planner_data.right_bound);
  }

  std::vector<bool> hard_constraints_results;
  auto transform_to_sampling_path = [](const PlanResult plan) {
    sampler_common::Path path;
    for (size_t i = 0; i < plan->points.size(); ++i) {
      // plan->points[i].point.pose

      const auto x = plan->points[i].point.pose.position.x;
      const auto y = plan->points[i].point.pose.position.y;
      const auto quat = plan->points[i].point.pose.orientation;
      // tf2::Quaternion quaternion(q.x,q.y,q.z,q.w);
      geometry_msgs::msg::Vector3 rpy;
      tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
      tf2::Matrix3x3(q).getRPY(rpy.x, rpy.y, rpy.z);
      // const auto z = plan->points[0].point.pose.position.z;
      path.points.emplace_back(Point2d{x, y});
      path.poses.emplace_back(plan->points[i].point.pose);
      path.yaws.emplace_back(rpy.z);
    }
    return path;
  };
  sampler_common::Path reference_path =
    transform_to_sampling_path(getPreviousModuleOutput().reference_path);
  const auto footprint = sampler_common::constraints::buildFootprintPoints(
    reference_path, internal_params_->constraints);

  behavior_path_planner::HardConstraintsFunctionVector hard_constraints_reference_path;
  hard_constraints_reference_path.emplace_back(
    [](
      sampler_common::Path & path, const sampler_common::Constraints & constraints,
      const MultiPoint2d & footprint) -> bool {
      path.constraint_results.collision =
        !sampler_common::constraints::has_collision(footprint, constraints.obstacle_polygons);
      return path.constraint_results.collision;
    });
  behavior_path_planner::evaluateHardConstraints(
    reference_path, internal_params_->constraints, footprint, hard_constraints_reference_path,
    hard_constraints_results);
  return reference_path.constraints_satisfied;
}

bool SamplingPlannerModule::isExecutionReady() const
{
  return true;
}

SamplingPlannerData SamplingPlannerModule::createPlannerData(
  const PlanResult & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound) const
{
  // create planner data
  SamplingPlannerData data;
  // planner_data.header = path.header;
  auto points = path->points;
  data.left_bound = left_bound;
  data.right_bound = right_bound;
  data.ego_pose = planner_data_->self_odometry->pose.pose;
  data.ego_vel = planner_data_->self_odometry->twist.twist.linear.x;
  // data.ego_vel = ego_state_ptr_->twist.twist.linear.x;
  return data;
}

PathWithLaneId SamplingPlannerModule::convertFrenetPathToPathWithLaneID(
  const frenet_planner::Path frenet_path, const lanelet::ConstLanelets & lanelets,
  const double path_z)
{
  auto quaternion_from_rpy = [](double roll, double pitch, double yaw) -> tf2::Quaternion {
    tf2::Quaternion quaternion_tf2;
    quaternion_tf2.setRPY(roll, pitch, yaw);
    return quaternion_tf2;
  };

  // auto copy_point_information = []()

  PathWithLaneId path;
  const auto header = planner_data_->route_handler->getRouteHeader();
  const auto reference_path_ptr = getPreviousModuleOutput().reference_path;

  for (size_t i = 0; i < frenet_path.points.size(); ++i) {
    const auto & frenet_path_point_position = frenet_path.points.at(i);
    const auto & frenet_path_point_yaw = frenet_path.yaws.at(i);
    // const auto & frenet_path_point_velocity = frenet_path.points.;
    PathPointWithLaneId point{};
    point.point.pose.position.x = frenet_path_point_position.x();
    point.point.pose.position.y = frenet_path_point_position.y();
    point.point.pose.position.z = path_z;

    auto yaw_as_quaternion = quaternion_from_rpy(0.0, 0.0, frenet_path_point_yaw);
    point.point.pose.orientation.w = yaw_as_quaternion.getW();
    point.point.pose.orientation.x = yaw_as_quaternion.getX();
    point.point.pose.orientation.y = yaw_as_quaternion.getY();
    point.point.pose.orientation.z = yaw_as_quaternion.getZ();

    // put the lane that contain waypoints in lane_ids.
    bool is_in_lanes = false;
    for (const auto & lane : lanelets) {
      if (lanelet::utils::isInLanelet(point.point.pose, lane)) {
        point.lane_ids.push_back(lane.id());
        is_in_lanes = true;
      }
    }
    // If none of them corresponds, assign the previous lane_ids.
    if (!is_in_lanes && i > 0) {
      point.lane_ids = path.points.at(i - 1).lane_ids;
    }
    if (reference_path_ptr) {
      const auto idx = motion_utils::findFirstNearestIndexWithSoftConstraints(
        reference_path_ptr->points, point.point.pose);
      const auto & closest_point = reference_path_ptr->points[idx];
      point.point.longitudinal_velocity_mps = closest_point.point.longitudinal_velocity_mps;
      point.point.lateral_velocity_mps = closest_point.point.lateral_velocity_mps;
    }
    path.points.push_back(point);
  }
  return path;
}

void SamplingPlannerModule::prepareConstraints(
  sampler_common::Constraints & constraints,
  const PredictedObjects::ConstSharedPtr & predicted_objects,
  const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound) const
{
  constraints.obstacle_polygons = sampler_common::MultiPolygon2d();
  for (const auto & o : predicted_objects->objects)
    if (o.kinematics.initial_twist_with_covariance.twist.linear.x < 0.5)  // TODO(Maxime): param
      constraints.obstacle_polygons.push_back(tier4_autoware_utils::toPolygon2d(o));

  constraints.dynamic_obstacles = {};  // TODO(Maxime): not implemented

  // TODO(Maxime): directly use lines instead of polygon

  sampler_common::Polygon2d drivable_area_polygon;
  for (const auto & p : right_bound) {
    drivable_area_polygon.outer().emplace_back(p.x, p.y);
  }
  for (auto it = left_bound.rbegin(); it != left_bound.rend(); ++it)
    drivable_area_polygon.outer().emplace_back(it->x, it->y);

  if (drivable_area_polygon.outer().size() < 1) {
    return;
  }
  drivable_area_polygon.outer().push_back(drivable_area_polygon.outer().front());

  constraints.drivable_polygons = {drivable_area_polygon};
}

BehaviorModuleOutput SamplingPlannerModule::plan()
{
  const auto reference_path_ptr = getPreviousModuleOutput().reference_path;
  if (reference_path_ptr->points.empty()) {
    return {};
  }
  auto reference_spline = [&]() -> sampler_common::transform::Spline2D {
    std::vector<double> x;
    std::vector<double> y;
    x.reserve(reference_path_ptr->points.size());
    y.reserve(reference_path_ptr->points.size());
    for (const auto & point : reference_path_ptr->points) {
      x.push_back(point.point.pose.position.x);
      y.push_back(point.point.pose.position.y);
    }
    return {x, y};
  }();
  frenet_planner::FrenetState frenet_initial_state;
  frenet_planner::SamplingParameters sampling_parameters;

  const auto & pose = planner_data_->self_odometry->pose.pose;
  sampler_common::State initial_state =
    behavior_path_planner::getInitialState(pose, reference_spline);
  sampling_parameters =
    prepareSamplingParameters(initial_state, reference_spline, *internal_params_);

  auto set_frenet_state = [](
                            const sampler_common::State & initial_state,
                            const sampler_common::transform::Spline2D & reference_spline,
                            frenet_planner::FrenetState & frenet_initial_state)

  {
    frenet_initial_state.position = initial_state.frenet;
    const auto frenet_yaw = initial_state.heading - reference_spline.yaw(initial_state.frenet.s);
    const auto path_curvature = reference_spline.curvature(initial_state.frenet.s);
    constexpr auto delta_s = 0.001;
    frenet_initial_state.lateral_velocity =
      (1 - path_curvature * initial_state.frenet.d) * std::tan(frenet_yaw);
    const auto path_curvature_deriv =
      (reference_spline.curvature(initial_state.frenet.s + delta_s) - path_curvature) / delta_s;
    const auto cos_yaw = std::cos(frenet_yaw);
    if (cos_yaw == 0.0) {
      frenet_initial_state.lateral_acceleration = 0.0;
    } else {
      frenet_initial_state.lateral_acceleration =
        -(path_curvature_deriv * initial_state.frenet.d +
          path_curvature * frenet_initial_state.lateral_velocity) *
          std::tan(frenet_yaw) +
        ((1 - path_curvature * initial_state.frenet.d) / (cos_yaw * cos_yaw)) *
          (initial_state.curvature * ((1 - path_curvature * initial_state.frenet.d) / cos_yaw) -
           path_curvature);
    }
  };
  set_frenet_state(initial_state, reference_spline, frenet_initial_state);

  std::vector<DrivableLanes> drivable_lanes{};

  const auto & prev_module_path = getPreviousModuleOutput().path;
  const auto & current_lanes = utils::getCurrentLanesFromPath(*prev_module_path, planner_data_);
  // expand drivable lanes
  std::for_each(current_lanes.begin(), current_lanes.end(), [&](const auto & lanelet) {
    drivable_lanes.push_back(generateExpandDrivableLanes(lanelet, planner_data_));
  });

  {
    const auto left_bound =
      (utils::calcBound(planner_data_->route_handler, drivable_lanes, false, false, true));
    const auto right_bound =
      (utils::calcBound(planner_data_->route_handler, drivable_lanes, false, false, false));

    const auto sampling_planner_data =
      createPlannerData(planner_data_->prev_output_path, left_bound, right_bound);

    prepareConstraints(
      internal_params_->constraints, planner_data_->dynamic_object,
      sampling_planner_data.left_bound, sampling_planner_data.right_bound);
  }

  auto frenet_paths =
    frenet_planner::generatePaths(reference_spline, frenet_initial_state, sampling_parameters);
  // if (prev_sampling_path_) {
  //   const auto prev_path = prev_sampling_path_.value();
  //   frenet_paths.push_back(prev_path);
  // }

  // EXTEND prev path
  if (prev_sampling_path_ && prev_sampling_path_->lengths.size() > 1) {
    // Update previous path
    frenet_planner::Path prev_path_frenet = prev_sampling_path_.value();
    frenet_paths.push_back(prev_path_frenet);

    geometry_msgs::msg::Pose future_pose = prev_path_frenet.poses.back();
    const double length_path = lanelet::utils::getArcCoordinates(current_lanes, future_pose).length;
    const auto goal_pose = planner_data_->route_handler->getGoalPose();
    const double length_goal = lanelet::utils::getArcCoordinates(current_lanes, goal_pose).length;

    const double min_target_length = *std::min_element(
      internal_params_->sampling.target_lengths.begin(),
      internal_params_->sampling.target_lengths.end());

    // double x = prev_path_frenet.points.back().x();
    // double x_pose = pose.position.x;
    if (std::abs(length_goal - length_path) > min_target_length) {
      // sampler_common::State reuse_state;
      // reuse_state.curvature = reused_path->curvatures.back();
      // reuse_state.pose = reused_path->points.back();
      // reuse_state.heading = reused_path->yaws.back();
      // reuse_state.frenet = reference_spline.frenet(reuse_state.pose);
      auto quaternion_from_rpy = [](double roll, double pitch, double yaw) -> tf2::Quaternion {
        tf2::Quaternion quaternion_tf2;
        quaternion_tf2.setRPY(roll, pitch, yaw);
        return quaternion_tf2;
      };

      geometry_msgs::msg::Pose future_pose;
      future_pose.position.x = prev_path_frenet.points.back().x();
      future_pose.position.y = prev_path_frenet.points.back().y();
      future_pose.position.z = pose.position.z;

      const auto future_pose_quaternion =
        quaternion_from_rpy(0.0, 0.0, prev_path_frenet.yaws.back());
      future_pose.orientation.w = future_pose_quaternion.w();
      future_pose.orientation.x = future_pose_quaternion.x();
      future_pose.orientation.y = future_pose_quaternion.y();
      future_pose.orientation.z = future_pose_quaternion.z();
      sampler_common::State future_state =
        behavior_path_planner::getInitialState(future_pose, reference_spline);
      frenet_planner::FrenetState frenet_reuse_state;

      set_frenet_state(future_state, reference_spline, frenet_reuse_state);

      // frenet_reuse_state.position = prev_path_frenet.frenet_points.back();

      frenet_planner::SamplingParameters extension_sampling_parameters =
        prepareSamplingParameters(future_state, reference_spline, *internal_params_);
      auto extension_frenet_paths = frenet_planner::generatePaths(
        reference_spline, frenet_reuse_state, extension_sampling_parameters);
      for (auto & p : extension_frenet_paths) frenet_paths.push_back(prev_path_frenet.extend(p));
    }
  }

  SoftConstraintsInputs soft_constraints_input;
  soft_constraints_input.goal_pose = planner_data_->route_handler->getGoalPose();
  soft_constraints_input.ego_pose = planner_data_->self_odometry->pose.pose;
  soft_constraints_input.current_lanes = current_lanes;
  soft_constraints_input.reference_path = reference_path_ptr;
  soft_constraints_input.prev_module_path = prev_module_path;

  debug_data_.footprints.clear();
  std::vector<std::vector<bool>> hard_constraints_results_full;
  std::vector<std::vector<double>> soft_constraints_results_full;
  for (auto & path : frenet_paths) {
    std::vector<bool> hard_constraints_results;
    std::vector<double> soft_constraints_results;
    const auto footprint =
      sampler_common::constraints::buildFootprintPoints(path, internal_params_->constraints);
    behavior_path_planner::evaluateHardConstraints(
      path, internal_params_->constraints, footprint, hard_constraints_, hard_constraints_results);
    path.constraint_results.curvature = true;
    debug_data_.footprints.push_back(footprint);
    evaluateSoftConstraints(
      path, internal_params_->constraints, soft_constraints_, soft_constraints_input,
      soft_constraints_results);
    soft_constraints_results_full.push_back(soft_constraints_results);
  }

  std::vector<sampler_common::Path> candidate_paths;
  const auto move_to_paths = [&candidate_paths](auto & paths_to_move) {
    candidate_paths.insert(
      candidate_paths.end(), std::make_move_iterator(paths_to_move.begin()),
      std::make_move_iterator(paths_to_move.end()));
  };

  move_to_paths(frenet_paths);
  debug_data_.previous_sampled_candidates_nb = debug_data_.sampled_candidates.size();
  debug_data_.sampled_candidates = candidate_paths;
  debug_data_.obstacles = internal_params_->constraints.obstacle_polygons;
  updateDebugMarkers();

  const auto best_path_idx = [](const auto & paths) {
    auto min_cost = std::numeric_limits<double>::max();
    size_t best_path_idx = 0;
    for (auto i = 0LU; i < paths.size(); ++i) {
      if (paths[i].constraints_satisfied && paths[i].cost < min_cost) {
        best_path_idx = i;
        min_cost = paths[i].cost;
      }
    }
    return min_cost < std::numeric_limits<double>::max() ? std::optional<size_t>(best_path_idx)
                                                         : std::nullopt;
  };
  const auto selected_path_idx = best_path_idx(frenet_paths);

  if (!selected_path_idx) {
    BehaviorModuleOutput out;
    out.path = getPreviousModuleOutput().path;
    out.reference_path = getPreviousModuleOutput().reference_path;
    out.drivable_area_info = getPreviousModuleOutput().drivable_area_info;
    return out;
  }

  const auto best_path = frenet_paths[*selected_path_idx];

  std::cerr << "Soft constraints results of best: ";
  for (const auto result : soft_constraints_results_full[*selected_path_idx])
    std::cerr << result << ",";
  std::cerr << "\n";

  std::cerr << "Poses " << best_path.poses.size() << "\n";

  prev_sampling_path_ = best_path;

  const double max_length = *std::max_element(
    internal_params_->sampling.target_lengths.begin(),
    internal_params_->sampling.target_lengths.end());
  const auto road_lanes =
    utils::getExtendedCurrentLanes(planner_data_, max_length, max_length, false);
  auto out_path = convertFrenetPathToPathWithLaneID(
    best_path, road_lanes, soft_constraints_input.goal_pose.position.z);

  BehaviorModuleOutput out;
  out.path = std::make_shared<PathWithLaneId>(out_path);
  out.reference_path = reference_path_ptr;
  out.drivable_area_info = getPreviousModuleOutput().drivable_area_info;
  extendOutputDrivableArea(out);
  return out;
}

void SamplingPlannerModule::updateDebugMarkers()
{
  debug_marker_.markers.clear();
  info_marker_.markers.clear();

  const auto header = planner_data_->route_handler->getRouteHeader();
  visualization_msgs::msg::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = header.stamp;
  m.action = m.ADD;
  m.id = 0UL;
  m.type = m.LINE_STRIP;
  m.color.a = 1.0;
  m.scale.x = 0.02;
  m.ns = "candidates";
  for (const auto & c : debug_data_.sampled_candidates) {
    m.points.clear();
    for (const auto & p : c.points)
      m.points.push_back(geometry_msgs::msg::Point().set__x(p.x()).set__y(p.y()));
    if (c.constraint_results.isValid()) {
      m.color.g = 1.0;
      m.color.r = 0.0;
    } else {
      m.color.r = 1.0;
      m.color.g = 0.0;
    }
    debug_marker_.markers.push_back(m);
    info_marker_.markers.push_back(m);
    ++m.id;
  }
  m.ns = "footprint";
  m.id = 0UL;
  m.type = m.POINTS;
  m.points.clear();
  m.color.a = 1.0;
  m.color.r = 1.0;
  m.color.g = 0.0;
  m.color.b = 1.0;
  m.scale.y = 0.2;
  if (!debug_data_.footprints.empty()) {
    m.action = m.ADD;
    for (const auto & p : debug_data_.footprints[0]) {
      m.points.push_back(geometry_msgs::msg::Point().set__x(p.x()).set__y(p.y()));
    }

  } else {
    m.action = m.DELETE;
  }
  debug_marker_.markers.push_back(m);
  info_marker_.markers.push_back(m);
  ++m.id;
  // m.ns = "debug_path";
  // m.id = 0UL;
  // m.type = m.POINTS;
  // m.points.clear();
  // m.color.g = 1.0;
  // m.color.b = 0.0;
  // m.scale.y = 0.04;
  // if (!debug_data_.sampled_candidates.empty()) {
  //   m.action = m.ADD;
  //   for (const auto & p :
  //        debug_data_.sampled_candidates[debug_data_.sampled_candidates.size()].points)
  //     m.points.push_back(geometry_msgs::msg::Point().set__x(p.x()).set__y(p.y()));
  // } else {
  //   m.action = m.DELETE;
  // }
  // debug_marker_.markers.push_back(m);
  // info_marker_.markers.push_back(m);
  m.type = m.LINE_STRIP;
  m.ns = "obstacles";
  m.id = 0UL;
  m.color.r = 1.0;
  m.color.g = 0.0;
  m.color.b = 0.0;
  for (const auto & obs : debug_data_.obstacles) {
    m.points.clear();
    for (const auto & p : obs.outer())
      m.points.push_back(geometry_msgs::msg::Point().set__x(p.x()).set__y(p.y()));
    debug_marker_.markers.push_back(m);
    info_marker_.markers.push_back(m);
    ++m.id;
  }
  m.action = m.DELETE;
  m.ns = "candidates";
  for (m.id = debug_data_.sampled_candidates.size();
       static_cast<size_t>(m.id) < debug_data_.previous_sampled_candidates_nb; ++m.id) {
    debug_marker_.markers.push_back(m);
    info_marker_.markers.push_back(m);
  }
}

void SamplingPlannerModule::extendOutputDrivableArea(BehaviorModuleOutput & output)
{
  const auto prev_module_path = getPreviousModuleOutput().path;
  const auto current_lanes = utils::getCurrentLanesFromPath(*prev_module_path, planner_data_);

  std::vector<DrivableLanes> drivable_lanes{};
  // expand drivable lanes
  std::for_each(current_lanes.begin(), current_lanes.end(), [&](const auto & lanelet) {
    drivable_lanes.push_back(generateExpandDrivableLanes(lanelet, planner_data_));
  });

  // // const auto drivable_lanes = utils::lane_change::generateDrivableLanes(
  // //   *planner_data_->route_handler, current_lanes, status_.target_lanes);
  // const auto drivable_lanes = behavior_path_planner::utils::generateDrivableLanes(current_lanes);
  // const auto shorten_lanes = utils::cutOverlappedLanes(*output.path, drivable_lanes);
  // const auto expanded_lanes = utils::expandLanelets(
  //   shorten_lanes, dp.drivable_area_left_bound_offset, dp.drivable_area_right_bound_offset,
  //   dp.drivable_area_types_to_skip);

  // // for new architecture
  DrivableAreaInfo current_drivable_area_info;
  current_drivable_area_info.drivable_lanes = drivable_lanes;
  output.drivable_area_info = utils::combineDrivableAreaInfo(
    current_drivable_area_info, getPreviousModuleOutput().drivable_area_info);
}

CandidateOutput SamplingPlannerModule::planCandidate() const
{
  return {};
}

void SamplingPlannerModule::updateData()
{
}

// utils

template <typename T>
void pushUniqueVector(T & base_vector, const T & additional_vector)
{
  base_vector.insert(base_vector.end(), additional_vector.begin(), additional_vector.end());
}

bool SamplingPlannerModule::isEndPointsConnected(
  const lanelet::ConstLanelet & left_lane, const lanelet::ConstLanelet & right_lane) const
{
  const auto & left_back_point_2d = right_lane.leftBound2d().back().basicPoint();
  const auto & right_back_point_2d = left_lane.rightBound2d().back().basicPoint();

  constexpr double epsilon = 1e-5;
  return (right_back_point_2d - left_back_point_2d).norm() < epsilon;
}

DrivableLanes SamplingPlannerModule::generateExpandDrivableLanes(
  const lanelet::ConstLanelet & lanelet,
  const std::shared_ptr<const PlannerData> & planner_data) const
{
  const auto & route_handler = planner_data->route_handler;

  DrivableLanes current_drivable_lanes;
  current_drivable_lanes.left_lane = lanelet;
  current_drivable_lanes.right_lane = lanelet;

  // 1. get left/right side lanes
  const auto update_left_lanelets = [&](const lanelet::ConstLanelet & target_lane) {
    const auto all_left_lanelets =
      route_handler->getAllLeftSharedLinestringLanelets(target_lane, true, true);
    if (!all_left_lanelets.empty()) {
      current_drivable_lanes.left_lane = all_left_lanelets.back();  // leftmost lanelet
      pushUniqueVector(
        current_drivable_lanes.middle_lanes,
        lanelet::ConstLanelets(all_left_lanelets.begin(), all_left_lanelets.end() - 1));
    }
  };
  const auto update_right_lanelets = [&](const lanelet::ConstLanelet & target_lane) {
    const auto all_right_lanelets =
      route_handler->getAllRightSharedLinestringLanelets(target_lane, true, true);
    if (!all_right_lanelets.empty()) {
      current_drivable_lanes.right_lane = all_right_lanelets.back();  // rightmost lanelet
      pushUniqueVector(
        current_drivable_lanes.middle_lanes,
        lanelet::ConstLanelets(all_right_lanelets.begin(), all_right_lanelets.end() - 1));
    }
  };

  update_left_lanelets(lanelet);
  update_right_lanelets(lanelet);

  // 2.1 when there are multiple lanes whose previous lanelet is the same
  const auto get_next_lanes_from_same_previous_lane =
    [&route_handler](const lanelet::ConstLanelet & lane) {
      // get previous lane, and return false if previous lane does not exist
      lanelet::ConstLanelets prev_lanes;
      if (!route_handler->getPreviousLaneletsWithinRoute(lane, &prev_lanes)) {
        return lanelet::ConstLanelets{};
      }

      lanelet::ConstLanelets next_lanes;
      for (const auto & prev_lane : prev_lanes) {
        const auto next_lanes_from_prev = route_handler->getNextLanelets(prev_lane);
        pushUniqueVector(next_lanes, next_lanes_from_prev);
      }
      return next_lanes;
    };

  const auto next_lanes_for_right =
    get_next_lanes_from_same_previous_lane(current_drivable_lanes.right_lane);
  const auto next_lanes_for_left =
    get_next_lanes_from_same_previous_lane(current_drivable_lanes.left_lane);

  // 2.2 look for neighbor lane recursively, where end line of the lane is connected to end line
  // of the original lane
  const auto update_drivable_lanes =
    [&](const lanelet::ConstLanelets & next_lanes, const bool is_left) {
      for (const auto & next_lane : next_lanes) {
        const auto & edge_lane =
          is_left ? current_drivable_lanes.left_lane : current_drivable_lanes.right_lane;
        if (next_lane.id() == edge_lane.id()) {
          continue;
        }

        const auto & left_lane = is_left ? next_lane : edge_lane;
        const auto & right_lane = is_left ? edge_lane : next_lane;
        if (!isEndPointsConnected(left_lane, right_lane)) {
          continue;
        }

        if (is_left) {
          current_drivable_lanes.left_lane = next_lane;
        } else {
          current_drivable_lanes.right_lane = next_lane;
        }

        const auto & middle_lanes = current_drivable_lanes.middle_lanes;
        const auto has_same_lane = std::any_of(
          middle_lanes.begin(), middle_lanes.end(),
          [&edge_lane](const auto & lane) { return lane.id() == edge_lane.id(); });

        if (!has_same_lane) {
          if (is_left) {
            if (current_drivable_lanes.right_lane.id() != edge_lane.id()) {
              current_drivable_lanes.middle_lanes.push_back(edge_lane);
            }
          } else {
            if (current_drivable_lanes.left_lane.id() != edge_lane.id()) {
              current_drivable_lanes.middle_lanes.push_back(edge_lane);
            }
          }
        }

        return true;
      }
      return false;
    };

  const auto expand_drivable_area_recursively =
    [&](const lanelet::ConstLanelets & next_lanes, const bool is_left) {
      // NOTE: set max search num to avoid infinity loop for drivable area expansion
      constexpr size_t max_recursive_search_num = 3;
      for (size_t i = 0; i < max_recursive_search_num; ++i) {
        const bool is_update_kept = update_drivable_lanes(next_lanes, is_left);
        if (!is_update_kept) {
          break;
        }
        if (i == max_recursive_search_num - 1) {
          RCLCPP_ERROR(
            rclcpp::get_logger("behavior_path_planner").get_child("avoidance"),
            "Drivable area expansion reaches max iteration.");
        }
      }
    };
  expand_drivable_area_recursively(next_lanes_for_right, false);
  expand_drivable_area_recursively(next_lanes_for_left, true);

  // 3. update again for new left/right lanes
  update_left_lanelets(current_drivable_lanes.left_lane);
  update_right_lanelets(current_drivable_lanes.right_lane);

  // 4. compensate that current_lane is in either of left_lane, right_lane or middle_lanes.
  if (
    current_drivable_lanes.left_lane.id() != lanelet.id() &&
    current_drivable_lanes.right_lane.id() != lanelet.id()) {
    current_drivable_lanes.middle_lanes.push_back(lanelet);
  }

  return current_drivable_lanes;
}

frenet_planner::SamplingParameters SamplingPlannerModule::prepareSamplingParameters(
  const sampler_common::State & initial_state,
  const sampler_common::transform::Spline2D & path_spline,
  const SamplingPlannerInternalParameters & params_)
{
  // calculate target lateral positions
  std::vector<double> target_lateral_positions;
  if (params_.sampling.nb_target_lateral_positions > 1) {
    target_lateral_positions = {0.0, initial_state.frenet.d};
    double min_d = 0.0;
    double max_d = 0.0;
    double min_d_s = std::numeric_limits<double>::max();
    double max_d_s = std::numeric_limits<double>::max();
    for (const auto & drivable_poly : params_.constraints.drivable_polygons) {
      for (const auto & p : drivable_poly.outer()) {
        const auto frenet_coordinates = path_spline.frenet(p);
        const auto d_s = std::abs(frenet_coordinates.s - initial_state.frenet.s);
        if (d_s < min_d_s && frenet_coordinates.d < 0.0) {
          min_d_s = d_s;
          min_d = frenet_coordinates.d;
        }
        if (d_s < max_d_s && frenet_coordinates.d > 0.0) {
          max_d_s = d_s;
          max_d = frenet_coordinates.d;
        }
      }
    }
    min_d += params_.constraints.ego_width / 2.0;
    max_d -= params_.constraints.ego_width / 2.0;
    if (min_d < max_d) {
      for (auto r = 0.0; r <= 1.0; r += 1.0 / (params_.sampling.nb_target_lateral_positions - 1))
        target_lateral_positions.push_back(interpolation::lerp(min_d, max_d, r));
    }
  } else {
    target_lateral_positions = params_.sampling.target_lateral_positions;
  }
  frenet_planner::SamplingParameters sampling_parameters;
  sampling_parameters.resolution = params_.sampling.resolution;
  const auto max_s = path_spline.lastS();
  frenet_planner::SamplingParameter p;
  for (const auto target_length : params_.sampling.target_lengths) {
    p.target_state.position.s =
      std::min(max_s, path_spline.frenet(initial_state.pose).s + std::max(0.0, target_length));
    for (const auto target_lateral_position : target_lateral_positions) {
      p.target_state.position.d = target_lateral_position;
      for (const auto target_lat_vel : params_.sampling.frenet.target_lateral_velocities) {
        p.target_state.lateral_velocity = target_lat_vel;
        for (const auto target_lat_acc : params_.sampling.frenet.target_lateral_accelerations) {
          p.target_state.lateral_acceleration = target_lat_acc;
          sampling_parameters.parameters.push_back(p);
        }
      }
    }
    if (p.target_state.position.s == max_s) break;
  }
  return sampling_parameters;
}

}  // namespace behavior_path_planner
