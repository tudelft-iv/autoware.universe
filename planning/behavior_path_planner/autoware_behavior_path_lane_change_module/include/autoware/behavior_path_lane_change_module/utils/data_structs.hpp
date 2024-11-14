// Copyright 2022 TIER IV, Inc.
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
#ifndef AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__DATA_STRUCTS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__DATA_STRUCTS_HPP_

#include "autoware/behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp"
#include "autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"

#include <autoware/behavior_path_planner_common/parameters.hpp>
#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware/universe_utils/math/unit_conversion.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner::lane_change
{
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;
using route_handler::Direction;
using route_handler::RouteHandler;
using universe_utils::Polygon2d;
using utils::path_safety_checker::ExtendedPredictedObjects;

struct LateralAccelerationMap
{
  std::vector<double> base_vel;
  std::vector<double> base_min_acc;
  std::vector<double> base_max_acc;

  void add(const double velocity, const double min_acc, const double max_acc)
  {
    if (base_vel.size() != base_min_acc.size() || base_vel.size() != base_max_acc.size()) {
      return;
    }

    size_t idx = 0;
    for (size_t i = 0; i < base_vel.size(); ++i) {
      if (velocity < base_vel.at(i)) {
        break;
      }
      idx = i + 1;
    }

    base_vel.insert(base_vel.begin() + idx, velocity);
    base_min_acc.insert(base_min_acc.begin() + idx, min_acc);
    base_max_acc.insert(base_max_acc.begin() + idx, max_acc);
  }

  std::pair<double, double> find(const double velocity) const
  {
    if (!base_vel.empty() && velocity < base_vel.front()) {
      return std::make_pair(base_min_acc.front(), base_max_acc.front());
    }
    if (!base_vel.empty() && velocity > base_vel.back()) {
      return std::make_pair(base_min_acc.back(), base_max_acc.back());
    }

    const double min_acc = autoware::interpolation::lerp(base_vel, base_min_acc, velocity);
    const double max_acc = autoware::interpolation::lerp(base_vel, base_max_acc, velocity);

    return std::make_pair(min_acc, max_acc);
  }
};

struct CancelParameters
{
  bool enable_on_prepare_phase{true};
  bool enable_on_lane_changing_phase{false};
  double delta_time{1.0};
  double duration{5.0};
  double max_lateral_jerk{10.0};
  double overhang_tolerance{0.0};

  // unsafe_hysteresis_threshold will be compare with the number of detected unsafe instance. If the
  // number of unsafe exceeds unsafe_hysteresis_threshold, the lane change will be cancelled or
  // aborted.
  int unsafe_hysteresis_threshold{2};

  int deceleration_sampling_num{5};
};

struct Parameters
{
  // trajectory generation
  double backward_lane_length{200.0};
  double prediction_time_resolution{0.5};
  int longitudinal_acc_sampling_num{10};
  int lateral_acc_sampling_num{10};

  // lane change parameters
  double backward_length_buffer_for_end_of_lane{0.0};
  double backward_length_buffer_for_blocking_object{0.0};
  double backward_length_from_intersection{5.0};
  double lane_changing_lateral_jerk{0.5};
  double minimum_lane_changing_velocity{5.6};
  double lane_change_prepare_duration{4.0};
  LateralAccelerationMap lane_change_lat_acc_map;

  // parked vehicle
  double object_check_min_road_shoulder_width{0.5};
  double object_shiftable_ratio_threshold{0.6};

  // turn signal
  double min_length_for_turn_signal_activation{10.0};
  double length_ratio_for_turn_signal_deactivation{0.8};

  // acceleration data
  double min_longitudinal_acc{-1.0};
  double max_longitudinal_acc{1.0};

  double skip_process_lon_diff_th_prepare{0.5};
  double skip_process_lon_diff_th_lane_changing{1.0};

  // collision check
  bool enable_collision_check_for_prepare_phase_in_general_lanes{false};
  bool enable_collision_check_for_prepare_phase_in_intersection{true};
  bool enable_collision_check_for_prepare_phase_in_turns{true};
  double stopped_object_velocity_threshold{0.1};
  bool check_objects_on_current_lanes{true};
  bool check_objects_on_other_lanes{true};
  bool use_all_predicted_path{false};
  double lane_expansion_left_offset{0.0};
  double lane_expansion_right_offset{0.0};

  // regulatory elements
  bool regulate_on_crosswalk{false};
  bool regulate_on_intersection{false};
  bool regulate_on_traffic_light{false};

  // ego vehicle stuck detection
  double stop_velocity_threshold{0.1};
  double stop_time_threshold{3.0};

  // true by default for all objects
  utils::path_safety_checker::ObjectTypesToCheck object_types_to_check;

  // safety check
  bool allow_loose_check_for_cancel{true};
  bool enable_target_lane_bound_check{true};
  double collision_check_yaw_diff_threshold{3.1416};
  utils::path_safety_checker::RSSparams rss_params{};
  utils::path_safety_checker::RSSparams rss_params_for_parked{};
  utils::path_safety_checker::RSSparams rss_params_for_abort{};
  utils::path_safety_checker::RSSparams rss_params_for_stuck{};

  // abort
  CancelParameters cancel{};

  // finish judge parameter
  double lane_change_finish_judge_buffer{3.0};
  double finish_judge_lateral_threshold{0.2};
  double finish_judge_lateral_angle_deviation{autoware::universe_utils::deg2rad(3.0)};

  // debug marker
  bool publish_debug_marker{false};
};

enum class States {
  Normal = 0,
  Cancel,
  Abort,
  Stop,
};

struct PhaseInfo
{
  double prepare{0.0};
  double lane_changing{0.0};

  [[nodiscard]] double sum() const { return prepare + lane_changing; }

  PhaseInfo(const double _prepare, const double _lane_changing)
  : prepare(_prepare), lane_changing(_lane_changing)
  {
  }
};

struct PhaseMetrics
{
  double duration{0.0};
  double length{0.0};
  double velocity{0.0};
  double sampled_lon_accel{0.0};
  double actual_lon_accel{0.0};
  double lat_accel{0.0};

  PhaseMetrics(
    const double _duration, const double _length, const double _velocity,
    const double _sampled_lon_accel, const double _actual_lon_accel, const double _lat_accel)
  : duration(_duration),
    length(_length),
    velocity(_velocity),
    sampled_lon_accel(_sampled_lon_accel),
    actual_lon_accel(_actual_lon_accel),
    lat_accel(_lat_accel)
  {
  }
};

struct Lanes
{
  bool current_lane_in_goal_section{false};
  bool target_lane_in_goal_section{false};
  lanelet::ConstLanelet ego_lane;
  lanelet::ConstLanelets current;
  lanelet::ConstLanelets target_neighbor;
  lanelet::ConstLanelets target;
  std::vector<lanelet::ConstLanelets> preceding_target;
};

struct Info
{
  PhaseInfo longitudinal_acceleration{0.0, 0.0};
  PhaseInfo velocity{0.0, 0.0};
  PhaseInfo duration{0.0, 0.0};
  PhaseInfo length{0.0, 0.0};

  Pose lane_changing_start{};
  Pose lane_changing_end{};

  ShiftLine shift_line{};

  double lateral_acceleration{0.0};
  double terminal_lane_changing_velocity{0.0};

  Info() = default;
  Info(
    const PhaseMetrics & _prep_metrics, const PhaseMetrics & _lc_metrics,
    const Pose & _lc_start_pose, const Pose & _lc_end_pose, const ShiftLine & _shift_line)
  {
    longitudinal_acceleration =
      PhaseInfo{_prep_metrics.actual_lon_accel, _lc_metrics.actual_lon_accel};
    duration = PhaseInfo{_prep_metrics.duration, _lc_metrics.duration};
    velocity = PhaseInfo{_prep_metrics.velocity, _prep_metrics.velocity};
    length = PhaseInfo{_prep_metrics.length, _lc_metrics.length};
    lane_changing_start = _lc_start_pose;
    lane_changing_end = _lc_end_pose;
    lateral_acceleration = _lc_metrics.lat_accel;
    terminal_lane_changing_velocity = _lc_metrics.velocity;
    shift_line = _shift_line;
  }
};

template <typename Object>
struct LanesObjects
{
  Object current_lane{};
  Object target_lane_leading{};
  Object target_lane_trailing{};
  Object other_lane{};

  LanesObjects() = default;
  LanesObjects(
    Object current_lane, Object target_lane_leading, Object target_lane_trailing, Object other_lane)
  : current_lane(std::move(current_lane)),
    target_lane_leading(std::move(target_lane_leading)),
    target_lane_trailing(std::move(target_lane_trailing)),
    other_lane(std::move(other_lane))
  {
  }
};

struct TargetObjects
{
  ExtendedPredictedObjects leading;
  ExtendedPredictedObjects trailing;
  TargetObjects(ExtendedPredictedObjects leading, ExtendedPredictedObjects trailing)
  : leading(std::move(leading)), trailing(std::move(trailing))
  {
  }
};

enum class ModuleType {
  NORMAL = 0,
  EXTERNAL_REQUEST,
  AVOIDANCE_BY_LANE_CHANGE,
};

struct PathSafetyStatus
{
  bool is_safe{true};
  bool is_trailing_object{false};

  PathSafetyStatus() = default;
  PathSafetyStatus(const bool is_safe, const bool is_trailing_object)
  : is_safe(is_safe), is_trailing_object(is_trailing_object)
  {
  }
};

struct LanesPolygon
{
  lanelet::BasicPolygon2d current;
  lanelet::BasicPolygon2d target;
  lanelet::BasicPolygon2d expanded_target;
  lanelet::BasicPolygon2d target_neighbor;
  std::vector<lanelet::BasicPolygon2d> preceding_target;
};

struct MinMaxValue
{
  double min{std::numeric_limits<double>::infinity()};
  double max{std::numeric_limits<double>::infinity()};
};

struct TransientData
{
  Polygon2d current_footprint;  // ego's polygon at current pose

  MinMaxValue lane_changing_length;  // lane changing length for a single lane change
  MinMaxValue
    current_dist_buffer;  // distance buffer computed backward from current lanes' terminal end
  MinMaxValue
    next_dist_buffer;  // distance buffer computed backward  from target lanes' terminal end
  double dist_to_terminal_end{
    std::numeric_limits<double>::min()};  // distance from ego base link to the current lanes'
                                          // terminal end
  double dist_from_prev_intersection{std::numeric_limits<double>::max()};
  // terminal end
  double dist_to_terminal_start{
    std::numeric_limits<double>::min()};  // distance from ego base link to the current lanes'
                                          // terminal start
  double max_prepare_length{
    std::numeric_limits<double>::max()};  // maximum prepare length, starting from ego's base link

  double target_lane_length{std::numeric_limits<double>::min()};

  lanelet::ArcCoordinates current_lanes_ego_arc;  // arc coordinates of ego pose along current lanes
  lanelet::ArcCoordinates target_lanes_ego_arc;   // arc coordinates of ego pose along target lanes

  size_t current_path_seg_idx;   // index of nearest segment to ego along current path
  double current_path_velocity;  // velocity of the current path at the ego position along the path

  bool is_ego_near_current_terminal_start{false};
  bool is_ego_stuck{false};

  bool in_turn_direction_lane{false};
  bool in_intersection{false};
};

using RouteHandlerPtr = std::shared_ptr<RouteHandler>;
using BppParamPtr = std::shared_ptr<BehaviorPathPlannerParameters>;
using LCParamPtr = std::shared_ptr<Parameters>;
using LanesPtr = std::shared_ptr<Lanes>;
using LanesPolygonPtr = std::shared_ptr<LanesPolygon>;

struct CommonData
{
  RouteHandlerPtr route_handler_ptr;
  Odometry::ConstSharedPtr self_odometry_ptr;
  BppParamPtr bpp_param_ptr;
  LCParamPtr lc_param_ptr;
  LanesPtr lanes_ptr;
  LanesPolygonPtr lanes_polygon_ptr;
  TransientData transient_data;
  PathWithLaneId current_lanes_path;
  ModuleType lc_type;
  Direction direction;

  [[nodiscard]] const Pose & get_ego_pose() const { return self_odometry_ptr->pose.pose; }

  [[nodiscard]] const Twist & get_ego_twist() const { return self_odometry_ptr->twist.twist; }

  [[nodiscard]] double get_ego_speed(bool use_norm = false) const
  {
    if (!use_norm) {
      return get_ego_twist().linear.x;
    }

    const auto x = get_ego_twist().linear.x;
    const auto y = get_ego_twist().linear.y;
    return std::hypot(x, y);
  }

  [[nodiscard]] bool is_data_available() const
  {
    return route_handler_ptr && self_odometry_ptr && bpp_param_ptr && lc_param_ptr && lanes_ptr &&
           lanes_polygon_ptr;
  }

  [[nodiscard]] bool is_lanes_available() const
  {
    return lanes_ptr && !lanes_ptr->current.empty() && !lanes_ptr->target.empty() &&
           !lanes_ptr->target_neighbor.empty();
  }
};
using CommonDataPtr = std::shared_ptr<CommonData>;
}  // namespace autoware::behavior_path_planner::lane_change

namespace autoware::behavior_path_planner
{
using autoware_perception_msgs::msg::PredictedObject;
using utils::path_safety_checker::ExtendedPredictedObjects;
using LaneChangeModuleType = lane_change::ModuleType;
using LaneChangeParameters = lane_change::Parameters;
using LaneChangeStates = lane_change::States;
using LaneChangePhaseInfo = lane_change::PhaseInfo;
using LaneChangePhaseMetrics = lane_change::PhaseMetrics;
using LaneChangeInfo = lane_change::Info;
using FilteredByLanesObjects = lane_change::LanesObjects<std::vector<PredictedObject>>;
using FilteredByLanesExtendedObjects = lane_change::LanesObjects<ExtendedPredictedObjects>;
using LateralAccelerationMap = lane_change::LateralAccelerationMap;
}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__DATA_STRUCTS_HPP_
