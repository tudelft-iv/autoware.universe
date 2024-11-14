// Copyright 2021 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__UTILS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__UTILS_HPP_

#include "autoware/behavior_path_lane_change_module/utils/data_structs.hpp"
#include "autoware/behavior_path_lane_change_module/utils/path.hpp"
#include "autoware/behavior_path_planner_common/parameters.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"
#include "rclcpp/logger.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware/universe_utils/geometry/boost_geometry.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <lanelet2_core/Forward.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner::utils::lane_change
{
using autoware::behavior_path_planner::utils::path_safety_checker::ExtendedPredictedObject;
using autoware::behavior_path_planner::utils::path_safety_checker::
  PoseWithVelocityAndPolygonStamped;
using autoware::behavior_path_planner::utils::path_safety_checker::PoseWithVelocityStamped;
using autoware::behavior_path_planner::utils::path_safety_checker::PredictedPathWithPolygon;
using autoware::route_handler::Direction;
using autoware::universe_utils::LineString2d;
using autoware::universe_utils::Polygon2d;
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::PredictedPath;
using behavior_path_planner::lane_change::CommonDataPtr;
using behavior_path_planner::lane_change::LanesPolygon;
using behavior_path_planner::lane_change::ModuleType;
using behavior_path_planner::lane_change::PathSafetyStatus;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using path_safety_checker::CollisionCheckDebugMap;
using tier4_planning_msgs::msg::PathWithLaneId;

bool is_mandatory_lane_change(const ModuleType lc_type);

double calcLaneChangeResampleInterval(
  const double lane_changing_length, const double lane_changing_velocity);

void setPrepareVelocity(
  PathWithLaneId & prepare_segment, const double current_velocity, const double prepare_velocity);

std::vector<int64_t> replaceWithSortedIds(
  const std::vector<int64_t> & original_lane_ids,
  const std::vector<std::vector<int64_t>> & sorted_lane_ids);

std::vector<std::vector<int64_t>> get_sorted_lane_ids(const CommonDataPtr & common_data_ptr);

lanelet::ConstLanelets get_target_neighbor_lanes(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & current_lanes,
  const LaneChangeModuleType & type);

bool isPathInLanelets(
  const PathWithLaneId & path, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes);

bool path_footprint_exceeds_target_lane_bound(
  const CommonDataPtr & common_data_ptr, const PathWithLaneId & path, const VehicleInfo & ego_info,
  const double margin = 0.1);

std::optional<LaneChangePath> construct_candidate_path(
  const CommonDataPtr & common_data_ptr, const LaneChangeInfo & lane_change_info,
  const PathWithLaneId & prepare_segment, const PathWithLaneId & target_lane_reference_path,
  const std::vector<std::vector<int64_t>> & sorted_lane_ids);

ShiftLine get_lane_changing_shift_line(
  const Pose & lane_changing_start_pose, const Pose & lane_changing_end_pose,
  const PathWithLaneId & reference_path, const double shift_length);

PathWithLaneId get_reference_path_from_target_Lane(
  const CommonDataPtr & common_data_ptr, const Pose & lane_changing_start_pose,
  const double lane_changing_length, const double resample_interval);

std::vector<DrivableLanes> generateDrivableLanes(
  const std::vector<DrivableLanes> & original_drivable_lanes, const RouteHandler & route_handler,
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & lane_change_lanes);

std::vector<DrivableLanes> generateDrivableLanes(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & lane_change_lanes);

double getLateralShift(const LaneChangePath & path);

CandidateOutput assignToCandidate(
  const LaneChangePath & lane_change_path, const Point & ego_position);

std::optional<lanelet::ConstLanelet> get_lane_change_target_lane(
  const CommonDataPtr & common_data_ptr, const lanelet::ConstLanelets & current_lanes);

std::vector<PoseWithVelocityStamped> convertToPredictedPath(
  const LaneChangePath & lane_change_path, const Twist & vehicle_twist, const Pose & pose,
  const double lane_changing_acceleration, const BehaviorPathPlannerParameters & common_parameters,
  const LaneChangeParameters & lane_change_parameters, const double resolution);

bool isParkedObject(
  const PathWithLaneId & path, const RouteHandler & route_handler,
  const ExtendedPredictedObject & object, const double object_check_min_road_shoulder_width,
  const double object_shiftable_ratio_threshold,
  const double static_object_velocity_threshold = 1.0);

bool isParkedObject(
  const lanelet::ConstLanelet & closest_lanelet, const lanelet::BasicLineString2d & boundary,
  const ExtendedPredictedObject & object, const double buffer_to_bound,
  const double ratio_threshold);

bool passed_parked_objects(
  const CommonDataPtr & common_data_ptr, const LaneChangePath & lane_change_path,
  const std::vector<ExtendedPredictedObject> & objects, CollisionCheckDebugMap & object_debug);

std::optional<size_t> getLeadingStaticObjectIdx(
  const RouteHandler & route_handler, const LaneChangePath & lane_change_path,
  const std::vector<ExtendedPredictedObject> & objects,
  const double object_check_min_road_shoulder_width, const double object_shiftable_ratio_threshold);

lanelet::BasicPolygon2d create_polygon(
  const lanelet::ConstLanelets & lanes, const double start_dist, const double end_dist);

ExtendedPredictedObject transform(
  const PredictedObject & object, const BehaviorPathPlannerParameters & common_parameters,
  const LaneChangeParameters & lane_change_parameters, const bool check_at_prepare_phase);

bool is_collided_polygons_in_lanelet(
  const std::vector<Polygon2d> & collided_polygons, const lanelet::BasicPolygon2d & lanes_polygon);

/**
 * @brief Generates expanded lanelets based on the given direction and offsets.
 *
 * Expands the provided lanelets in either the left or right direction based on
 * the specified direction. If the direction is 'LEFT', the lanelets are expanded
 * using the left_offset; if 'RIGHT', they are expanded using the right_offset.
 * Otherwise, no expansion occurs.
 *
 * @param lanes The lanelets to be expanded.
 * @param direction The direction of expansion: either LEFT or RIGHT.
 * @param left_offset The offset value for left expansion.
 * @param right_offset The offset value for right expansion.
 * @return lanelet::ConstLanelets A collection of expanded lanelets.
 */
lanelet::ConstLanelets generateExpandedLanelets(
  const lanelet::ConstLanelets & lanes, const Direction direction, const double left_offset,
  const double right_offset);

/**
 * @brief Retrieves a logger instance for a specific lane change type.
 *
 * This function provides a specialized logger for different types of lane change.
 *
 * @param type A string representing the type of lane change operation. This could be
 *             a specific maneuver or condition related to lane changing, such as
 *             'avoidance_by_lane_change', 'normal', 'external_request'.
 *
 * @return rclcpp::Logger The logger instance configured for the specified lane change type.
 */
rclcpp::Logger getLogger(const std::string & type);

/**
 * @brief Computes the current footprint of the ego vehicle based on its pose and size.
 *
 * This function calculates the 2D polygon representing the current footprint of the ego vehicle.
 * The footprint is determined by the vehicle's pose and its dimensions, including the distance
 * from the base to the front and rear ends of the vehicle, as well as its width.
 *
 * @param common_data_ptr Shared pointer to CommonData that holds necessary ego vehicle's dimensions
 *                        and pose information.
 *
 * @return Polygon2d A polygon representing the current 2D footprint of the ego vehicle.
 */
Polygon2d get_ego_footprint(const Pose & ego_pose, const VehicleInfo & ego_info);

Point getEgoFrontVertex(const Pose & ego_pose, const VehicleInfo & ego_info, bool left);

/**
 * @brief Checks if the given polygon is within an intersection area.
 *
 * This function evaluates whether a specified polygon is located within the bounds of an
 * intersection. It identifies the intersection area by checking the attributes of the provided
 * lanelet. If the lanelet has an attribute indicating it is part of an intersection, the function
 * then checks if the polygon is fully contained within this area.
 *
 * @param route_handler a shared pointer to the route_handler
 * @param lanelet A lanelet to check against the
 *                intersection area.
 * @param polygon The polygon to check for containment within the intersection area.
 *
 * @return bool True if the polygon is within the intersection area, false otherwise.
 */
bool is_within_intersection(
  const std::shared_ptr<RouteHandler> & route_handler, const lanelet::ConstLanelet & lanelet,
  const Polygon2d & polygon);

/**
 * @brief Determines if a polygon is within lanes designated for turning.
 *
 * Checks if a polygon overlaps with lanelets tagged for turning directions (excluding 'straight').
 * It evaluates the lanelet's 'turn_direction' attribute and determines overlap with the lanelet's
 * area.
 *
 * @param lanelet Lanelet representing the road segment whose turn direction is to be evaluated.
 * @param polygon The polygon to be checked for its presence within turn direction lanes.
 *
 * @return bool True if the polygon is within a lane designated for turning, false if it is within a
 *              straight lane or no turn direction is specified.
 */
bool is_within_turn_direction_lanes(
  const lanelet::ConstLanelet & lanelet, const Polygon2d & polygon);

LanesPolygon create_lanes_polygon(const CommonDataPtr & common_data_ptr);

bool is_same_lane_with_prev_iteration(
  const CommonDataPtr & common_data_ptr, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes);

bool is_ahead_of_ego(
  const CommonDataPtr & common_data_ptr, const PathWithLaneId & path,
  const PredictedObject & object);

bool is_before_terminal(
  const CommonDataPtr & common_data_ptr, const PathWithLaneId & path,
  const PredictedObject & object);

double calc_angle_to_lanelet_segment(const lanelet::ConstLanelets & lanelets, const Pose & pose);

ExtendedPredictedObjects transform_to_extended_objects(
  const CommonDataPtr & common_data_ptr, const std::vector<PredictedObject> & objects,
  const bool check_prepare_phase);

double get_distance_to_next_regulatory_element(
  const CommonDataPtr & common_data_ptr, const bool ignore_crosswalk = false,
  const bool ignore_intersection = false);

/**
 * @brief Calculates the minimum distance to a stationary object in the current lanes.
 *
 * This function determines the closest distance from the ego vehicle to a stationary object
 * in the current lanes. It checks if the object is within the stopping criteria based on its
 * velocity and calculates the distance while accounting for the object's size. Only objects
 * positioned after the specified distance to the target lane's start are considered.
 *
 * @param common_data_ptr Pointer to the common data structure containing parameters for lane
 * change.
 * @param filtered_objects A collection of objects filtered by lanes, including those in the current
 * lane.
 * @param dist_to_target_lane_start The distance to the start of the target lane from the ego
 * vehicle.
 * @param path The current path of the ego vehicle, containing path points and lane information.
 * @return The minimum distance to a stationary object in the current lanes. If no valid object is
 * found, returns the maximum possible double value.
 */
double get_min_dist_to_current_lanes_obj(
  const CommonDataPtr & common_data_ptr, const FilteredByLanesExtendedObjects & filtered_objects,
  const double dist_to_target_lane_start, const PathWithLaneId & path);

/**
 * @brief Checks if there is an object in the target lane that influences the decision to insert a
 * stop point.
 *
 * This function determines whether any objects exist in the target lane that would affect
 * the decision to place a stop point behind a blocking object in the current lane.
 *
 * @param common_data_ptr Pointer to the common data structure containing parameters for the lane
 * change.
 * @param filtered_objects A collection of objects filtered by lanes, including those in the target
 * lane.
 * @param stop_arc_length The arc length at which the ego vehicle is expected to stop.
 * @param path The current path of the ego vehicle, containing path points and lane information.
 * @return true if there is an object in the target lane that influences the stop point decision;
 * otherwise, false.
 */
bool has_blocking_target_object(
  const CommonDataPtr & common_data_ptr, const FilteredByLanesExtendedObjects & filtered_objects,
  const double stop_arc_length, const PathWithLaneId & path);

/**
 * @brief Checks if the ego vehicle has passed any turn direction within an intersection.
 *
 * This function determines whether the ego vehicle has exited the intersection and
 * turn lane area based on its distance from the previous intersection. It considers
 * whether the ego vehicle is currently in an intersection and a turn lane.
 *
 * @param common_data_ptr Shared pointer to CommonData containing the transient data and
 *                        lane-change parameters required for the distance's comparison.
 *
 * @return true if the ego vehicle has passed the intersection turn direction, false otherwise.
 */
bool has_passed_intersection_turn_direction(const CommonDataPtr & common_data_ptr);

/**
 * @brief Retrieves the predicted paths of an object as 2D line strings.
 *
 * This function transforms each predicted path of an object into a LineString2d, representing
 * a 2D sequence of points. Each point in the path is extracted from the predicted path's
 * position and converted to a 2D point.
 *
 * @param object The predicted object whose paths will be converted into 2D line strings.
 *
 * @return std::vector<LineString2d> A vector of 2D line strings representing the predicted paths
 *                                   of the object.
 */
std::vector<LineString2d> get_line_string_paths(const ExtendedPredictedObject & object);

/**
 * @brief Determines if there is an object in the turn lane that could overtake the ego vehicle.
 *
 * This function checks for any trailing objects in the turn lane that may attempt to overtake
 * the ego vehicle. The check is only applicable if the ego vehicle is still within a certain
 * distance from the previous intersection's turn lane. It evaluates whether any of the predicted
 * paths or the initial polygon of trailing objects overlap with the target lane polygon.
 *
 * @param common_data_ptr Shared pointer to CommonData containing lane and polygon information
 *                        for the ego vehicle.
 * @param trailing_objects A collection of predicted objects trailing the ego vehicle.
 *
 * @return true if there is an object in the turn lane with a potential to overtake, false
 * otherwise.
 */
bool has_overtaking_turn_lane_object(
  const CommonDataPtr & common_data_ptr, const ExtendedPredictedObjects & trailing_objects);
}  // namespace autoware::behavior_path_planner::utils::lane_change
#endif  // AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__UTILS_HPP_
